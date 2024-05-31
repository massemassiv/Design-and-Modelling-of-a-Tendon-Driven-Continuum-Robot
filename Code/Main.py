import time
from pathlib import Path
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from robust_serial.utils import get_serial_ports

import arduino_kommunikation as ard
import sensor_kommunikation as sens
import styrning as st


def plot_coordinates(coordinates: np.ndarray, L: float):
    X = coordinates[0, :]
    Y = coordinates[1, :]
    Z = coordinates[2, :]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(X, Y, Z)
    ax.plot(0, 0, L, "ro")
    # ax.plot_trisurf(X, Y, Z, color="white", edgecolors="grey", alpha=0.5)
    # ax.scatter(X, Y, Z, c="red")
    plt.show()
    plt.plot(coordinates[0, :], coordinates[1, :])


def plotta_vinkel(path, L, theta):
    df = pd.read_csv(path)
    R = L / theta
    alfa = df["Roll"].to_numpy()
    beta = df["Pitch"].to_numpy()
    alfa[0] = 2 * np.pi
    beta[0] = 2 * np.pi

    gamma = np.arctan(np.tan(alfa) / np.tan(beta))
    xy = [R * np.cos(gamma), R * np.sin(gamma)]

    plt.plot(xy[0], xy[1])
    plt.show()


def acho_mange_plot(path, L, theta, gamma):
    df = pd.read_csv(path)
    new_path = path.parent.joinpath("theta_est.csv")
    R = L / theta
    alfa = df["Roll"].to_numpy() * np.pi / 180.0
    beta = df["Pitch"].to_numpy() * np.pi / 180.0
    # alfa[0] = 2 * np.pi
    # beta[0] = 2 * np.pi
    theta_est = np.arctan(1 / (np.sqrt(np.tan(alfa) ** 2 + np.tan(beta) ** 2)))
    gamma_est = np.arctan(np.tan(alfa) / np.tan(beta))

    angle_arr = np.array([alfa, beta, theta_est])
    pd.DataFrame(
        {
            "alfa": alfa * 180.0 / np.pi,
            "beta": beta * 180.0 / np.pi,
            "gamma": gamma * 180.0 / np.pi,
            "theta_est": theta_est * 180.0 / np.pi,
            "gamma_est": gamma_est * 180.0 / np.pi,
        }
    ).to_csv(new_path)


def step_wires(
    dL_vec,
    servo_serial,
    sensor_serial,
    angle_dict,
    debug_servo=False,
    debug_sensor=False,
    delay=0.1,
):

    ard.write_servo_orders(
        servo_serial,
        dL_vec,
        debug=debug_servo,
    )
    angle_vec = [0, 0, 0]
    pos_vec = [0, 0, 0]
    time.sleep(delay)
    sens.get_sensor_data(sensor_serial, pos_vec, angle_vec, debug=debug_sensor)
    for jj, key in enumerate(angle_dict.keys()):
        angle_dict[key].append(angle_vec[jj])
    time.sleep(delay)


def goto_pos_and_meassure_n(
    dL_vec,
    n_meassurements,
    servo_serial,
    sensor_serial,
    meassurement_arr: Optional[np.ndarray] = None,
    inplace: Optional[bool] = False,
    debug_servo: Optional[bool] = False,
    debug_sensor: Optional[bool] = False,
    delays: Optional[List[float]] = [0, 0],
):
    if meassurement_arr is None:
        meassurement_arr = np.zeros((3, n_meassurements))
    ard.write_servo_orders(
        servo_serial, dL_vec, debug=debug_servo, read_delay=delays[0]
    )

    if not inplace:
        return get_n_meassurements(
            sensor_serial,
            n_meassurements=n_meassurements,
            meassurement_arr=meassurement_arr,
            inplace=inplace,
            delay=delays[1],
        )
    else:
        get_n_meassurements(
            sensor_serial,
            n_meassurements=n_meassurements,
            meassurement_arr=meassurement_arr,
            inplace=inplace,
            delay=delays[1],
        )


def perform_test(
    theta_arr,
    gamma_arr,
    servo_serial,
    sensor_serial,
    debug_servo,
    debug_sensor,
    delays,
    n_meassurements: Optional[int] = 100,
):
    assert len(theta_arr) == len(gamma_arr)
    L = 143e-3
    d = 12.54 * 1e-3
    m_points = len(theta_arr)
    meassurement_arr = np.zeros((3, n_meassurements, m_points))
    dL_arr = st.theta_gamma_to_delta_wire(theta_arr, gamma_arr, d)
    df_main = pd.DataFrame(
        columns=[
            "Gamma Ordered",
            "Theta Ordered",
            "point_index",
            "Roll",
            "Pitch",
            "Yaw",
        ]
    )
    for i in range(0, m_points):
        theta, gamma = theta_arr[i], gamma_arr[i]
        goto_pos_and_meassure_n(
            dL_arr[:, i],
            n_meassurements,
            servo_serial,
            sensor_serial,
            meassurement_arr[:, :, i],
            inplace=True,
            debug_servo=debug_servo,
            debug_sensor=debug_sensor,
            delays=delays,
        )
        temp_df = pd.DataFrame(
            meassurement_arr[:, :, i].T, columns=["Roll", "Pitch", "Yaw"]
        )
        temp_df["Gamma Ordered"] = np.rad2deg(gamma)
        temp_df["Theta Ordered"] = np.rad2deg(theta)
        temp_df["point_index"] = i
        if i == 0:
            df_main = temp_df
        else:
            df_main = pd.concat([df_main, temp_df], axis=0)
    return df_main


def test_const_theta(
    theta,
    gamma_start,
    gamma_end,
    m_points,
    servo_serial,
    sensor_serial,
    debug_servo,
    debug_sensor,
    delays,
    n_meassurements,
):
    theta_arr = np.ones(m_points) * np.deg2rad(theta)
    gamma_arr = np.linspace(gamma_start, gamma_end, m_points)
    df_main = perform_test(
        theta_arr,
        gamma_arr,
        servo_serial,
        sensor_serial,
        debug_servo,
        debug_sensor,
        delays,
        n_meassurements,
    )
    return df_main


def test_variable_theta(
    theta_start,
    theta_end,
    m_points,
    gamma,
    servo_serial,
    sensor_serial,
    debug_servo,
    debug_sensor,
    delays,
    n_meassurements,
):
    theta_values = np.linspace(np.deg2rad(theta_start), np.deg2rad(theta_end), m_points)
    theta_arr = np.zeros(2 * m_points)
    for i in range(0, m_points, 1):
        theta_arr[2 * i] = theta_values[i]
        theta_arr[2 * i + 1] = -theta_values[i]
    gamma_arr = np.ones(2 * m_points) * gamma
    df_main = perform_test(
        theta_arr,
        gamma_arr,
        servo_serial,
        sensor_serial,
        debug_servo,
        debug_sensor,
        delays,
        n_meassurements,
    )
    return df_main


def theta_est(alfa_arr, beta_arr):
    return np.arctan(1 / (np.sqrt(np.tan(alfa_arr) ** 2 + np.tan(beta_arr) ** 2)))


def gamma_est(alfa_arr, beta_arr):
    return np.arctan(np.tan(alfa_arr) / np.tan(beta_arr))


def get_n_meassurements(
    ser_file,
    n_meassurements: int,
    meassurement_arr: Optional[np.ndarray] = None,
    inplace: Optional[bool] = False,
    delay=0.01,
):
    angle_vec = [0, 0, 0]
    pos_vec = [0, 0, 0]
    if meassurement_arr is None:
        meassurement_arr = np.zeros((3, n_meassurements))
    for i in range(0, n_meassurements):
        angle_vec = [0, 0, 0]
        sens.get_sensor_data(ser_file, pos_vec, angle_vec, debug=False)

        meassurement_arr[:, i] = angle_vec
        time.sleep(delay)
    if not inplace:
        return meassurement_arr


if __name__ == "__main__":

    debug_servo = True
    debug_sensor = False
    available_ports = get_serial_ports()
    print(available_ports)
    # Serial files to use, ser_file1 is the servo and ser_file2 is the sensor.
    ser_file1 = ard.connect_to_arduino(serial_port=available_ports[1])

    ser_file2 = ard.connect_to_arduino(serial_port=available_ports[0])
    # File paths for saving the data.
    data_folder = Path(__file__).parent.joinpath("data")

    const_path = data_folder.joinpath("const_theta.csv")

    variable_path = data_folder.joinpath("variable_theta.csv")

    # Number of meassurements to take for each point.
    n_meassurements = 50

    # Parameters for test with constant theta and varying gamma.
    theta_const = 20

    gamma_start = 0
    gamma_end = 2 * np.pi
    m_const = 20  # Number of points to split the circle into

    # Parameters for test with constant gamma and varying theta.
    theta_start = 40
    theta_end = 80
    gamma_const = np.deg2rad(-40)  # 4 * np.pi / 3
    m_variable = 10  # Number of points to split the circle into

    # Perform the test with constant theta and save the data to csv files.
    df_const = test_const_theta(
        theta_const,
        gamma_start,
        gamma_end,
        m_const,
        ser_file1,
        ser_file2,
        debug_servo,
        debug_sensor,
        delays=[0, 0.1],
        n_meassurements=n_meassurements,
    )
    df_const.to_csv(const_path)

    # Perform the test with varying theta and save the data to csv files.
    df_variable = test_variable_theta(
        theta_start,
        theta_end,
        m_variable,
        gamma_const,
        ser_file1,
        ser_file2,
        debug_servo,
        debug_sensor,
        delays=[0, 0],
        n_meassurements=n_meassurements,
    )
    df_variable.to_csv(variable_path)
