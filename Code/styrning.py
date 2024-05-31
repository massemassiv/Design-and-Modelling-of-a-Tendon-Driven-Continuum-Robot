import numpy as np


def points_on_circle(radius: float, n_points: int = 100, z: float = 500):
    """Generates a list of points on a circle in the xy-plane with the center in origo.

    Args:
        radius (float): radius of the circle
        n_points (int, optional): number of points on the circle. Defaults to 100.
        z (float, optional): z height of the circle. Defaults to 500.

    Returns:
        _type_: _description_
    """
    return np.array(
        [
            [
                np.cos(2 * np.pi / n_points * i) * radius,
                np.sin(2 * np.pi / n_points * i) * radius,
                z,
            ]
            for i in range(n_points)
        ]
    )


def xyz_to_theta_beta(xyz_array: np.single):
    pass


def theta_beta_to_xyz(theta_beta_array: np.single, L):
    return np.array(
        [
            (L / theta_beta_array[0, 0])
            * (1 - np.cos(theta_beta_array[0, 0]))
            * np.cos(theta_beta_array[0, 1]),
            (L / theta_beta_array[0, 0])
            * (1 - np.cos(theta_beta_array[0, 0]))
            * np.sin(theta_beta_array[0, 1]),
            (L / theta_beta_array[0, 0]) * (np.sin(theta_beta_array[0, 0])),
        ]
    )


def create_theta_beta_arr(
    theta: float,
    n_points: int = 100,
    theta_start: float = 0,
    theta_end: float = 2 * np.pi,
):
    return np.array(
        [
            [
                np.ones(n_points - 1) * theta,
                np.diff(
                    np.linspace(theta_start, theta_end, num=n_points, dtype=np.single)
                ),
            ]
        ]
    )


SERVO_CONV = 1


def alpha_beta_to_delta_wire(theta_beta_arr: np.single, L: float, d: float):

    return np.array(
        [
            d * theta_beta_arr[0, 0] * np.cos(theta_beta_arr[0, 1]) * SERVO_CONV,
            d
            * theta_beta_arr[0, 0]
            * np.cos(theta_beta_arr[0, 1] + 2 * np.pi / 3)
            * SERVO_CONV,
            d
            * theta_beta_arr[0, 0]
            * np.cos(theta_beta_arr[0, 1] + 4 * np.pi / 3)
            * SERVO_CONV,
        ]
    )


def theta_gamma_to_delta_wire(theta_arr, gamma_arr, d: float):
    arr = np.array(
        [
            d * theta_arr * np.cos(gamma_arr),
            d * theta_arr * np.cos(gamma_arr + 2 * np.pi / 3),
            d * theta_arr * np.cos(gamma_arr + 4 * np.pi / 3),
        ]
    )
    print(arr)
    return arr


if __name__ == "__main__":
    L = 500 * 1e-3
    d = 25 * 1e-3
    theta = np.deg2rad(60)
    theta_beta_arr = create_theta_beta_arr(theta)
    xyz_arr = theta_beta_to_xyz(theta_beta_arr, L)
    delta_wire_arr = alpha_beta_to_delta_wire(theta_beta_arr, L, d)
    print(theta_beta_arr[0, 0])
    print(np.shape(delta_wire_arr))
    print(delta_wire_arr[:, 0])
    print(delta_wire_arr[0, 0])
    print(delta_wire_arr[1, 0])
    print(delta_wire_arr[2, 0])
