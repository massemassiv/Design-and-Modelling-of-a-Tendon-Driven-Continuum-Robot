import time
from typing import BinaryIO, List

import numpy as np
from robust_serial import read_i8, read_i16, read_i32, write_i8, write_i32
from robust_serial.utils import open_serial_port

from egna_orders import Order


def read_order(f: BinaryIO) -> Order:
    """
    :param f: file handler or serial file
    :return: (Order Enum Object)
    """
    return Order(read_i8(f))


def write_order(f: BinaryIO, order: Order) -> None:
    """
    :param f: file handler or serial file
    :param order: (Order Enum Object)
    """
    write_i8(f, order.value)


def get_sensor_data2(
    serial_file, position_vec: List, angle_vec: List, debug: bool = False
):
    position_order = [Order.SENT_POS_X, Order.SENT_POS_Y, Order.SENT_POS_Z]
    angle_order = [
        Order.SENT_ANGLE_X,
        Order.SENT_ANGLE_Y,
        Order.SENT_ANGLE_Z,
    ]
    write_order(serial_file, Order.GET_SENSOR)
    wait_for_order(serial_file, Order.RECEIVED, debug=debug)
    time.sleep(0.01)
    if debug:
        print("get_sensor_data")

    for i, w_order in enumerate(position_order):
        wait_for_order(serial_file, w_order, debug=debug)
        # order = read_order(serial_file)
        position_vec[i] = read_i16(serial_file)
        write_order(serial_file, Order.RECEIVED)
        time.sleep(0.1)
    if debug:

        print("position")
        # print(f"ORDER{i}: ", order)
        print("AccX", position_vec[0])
        print("AccY", position_vec[1])
        print("AccZ", position_vec[2])
    for i, w_order in enumerate(angle_order):
        # order = read_order(serial_file)
        wait_for_order(serial_file, w_order, debug=debug)
        angle_vec[i] = read_i16(serial_file)
        write_order(serial_file, Order.RECEIVED)
        time.sleep(0.1)
    if debug:

        print("position")
        # print(f"ORDER{i}: ", order)
        print("AngX", angle_vec[0])
        print("AngY", angle_vec[1])
        print("AngZ", angle_vec[2])


def get_sensor_data(
    serial_file, position_vec: List, angle_vec: List, debug: bool = False
):
    write_order(serial_file, Order.MOTOR)
    wait_for_received(serial_file, debug=debug)
    time.sleep(0.01)
    if debug:
        print("get_sensor_data")
    for i in range(3):
        wait_for_order(serial_file, Order.MOTOR, debug=debug)
        # order = read_order(serial_file)
        position_vec[i] = read_i16(serial_file)
        write_order(serial_file, Order.RECEIVED)
        time.sleep(0.1)
    if debug:

        print("position")
        # print(f"ORDER{i}: ", order)
        print("AccX", position_vec[0])
        print("AccY", position_vec[1])
        print("AccZ", position_vec[2])
    for i in range(3):
        # order = read_order(serial_file)
        wait_for_order(serial_file, Order.MOTOR, debug=debug)
        angle_vec[i] = read_i16(serial_file)
        write_order(serial_file, Order.RECEIVED)
        time.sleep(0.1)
    if debug:

        print("position")
        # print(f"ORDER{i}: ", order)
        print("AngX", angle_vec[0])
        print("AngY", angle_vec[1])
        print("AngZ", angle_vec[2])


def wait_for_received(serial_file, debug: bool = False):
    order = None
    while order != Order.RECEIVED:
        try:
            order = read_order(serial_file)
            if debug:
                print("ORDER_WAIT: ", order)
            if order == Order.SERVO:
                data = read_i16(serial_file)
                if debug:
                    print("DATA_WAIT: ", data)
            elif order == Order.HELLO or order == Order.ALREADY_CONNECTED:
                write_order(serial_file, Order.ALREADY_CONNECTED)

        except ValueError as e:
            if debug:
                print(e)
            else:
                pass

        time.sleep(0.01)


def wait_for_order(serial_file, wait_order, debug: bool = False, read_delay=0.01):
    order = None
    while order != wait_order:
        try:
            order = read_order(serial_file)
            if debug:
                print("ORDER_WAIT: ", order)
            if order == order.ERROR:
                print("ERROR")
                read_i16(serial_file)
                break
            elif order == Order.SERVO or order in [
                Order.SENT_SERVO1,
                Order.SENT_SERVO2,
                Order.SENT_SERVO3,
            ]:
                data = read_i32(serial_file)
                if debug:
                    print("DATA_WAIT: ", data)
            elif order == Order.HELLO or order == Order.ALREADY_CONNECTED:
                write_order(serial_file, Order.ALREADY_CONNECTED)

        except ValueError as e:
            if debug:
                print(e)
            else:
                pass
    # Bortagen 30/4
    time.sleep(read_delay)


def write_servo_orders3(  # OG write_servo_orders
    serial_file, servo_distances: List[int], debug: bool = False
) -> None:
    # order = None
    servo_distances = dlength_to_angle(servo_distances, debug=debug)
    # motor_adjustment = [1.212577713, 1.234914144, 1.228465357]
    # servo_distances = [int(servo_distances[i] * motor_adjustment[i]) for i in range(3)]
    for i in range(3):
        order = None
        # order_debug = None

        while order != Order.RECEIVED:  # and order_debug != Order.RECEIVED:
            if debug:
                print("i2=", i)
            write_order(serial_file, Order.SERVO)
            write_i32(serial_file, servo_distances[i])

            # wait_for_received(serial_file, debug=debug)
            # Så man kan ta bort wait_for_received
            wait_for_order(serial_file, Order.RECEIVED, debug=debug)
            order = Order.RECEIVED
            # order_debug = Order.RECEIVED

            # Bortagen 30/4
            # time.sleep(0.2)


def write_servo_orders(
    serial_file, servo_distances: List[int], debug: bool = False, read_delay=0.01
) -> None:
    # order = None
    sent_orders = (Order.SENT_SERVO1, Order.SENT_SERVO2, Order.SENT_SERVO3)
    received_orders = (
        Order.RECEIVED_SERVO1,
        Order.RECEIVED_SERVO2,
        Order.RECEIVED_SERVO3,
    )
    servo_distances = dlength_to_angle(servo_distances, debug=debug)
    write_order(serial_file, Order.SERVO)
    wait_for_order(
        serial_file, wait_order=Order.RECEIVED, debug=debug, read_delay=read_delay
    )
    if debug:
        pass
        # print("Write servo orders")
    for i, (r_order, s_order) in enumerate(zip(received_orders, sent_orders)):
        if debug:
            # print("i=", i)
            # print("r_order=", r_order)
            # print("s_order=", s_order)
            print(f"servo_distances[{i}]=", servo_distances[i])
        write_order(serial_file, s_order)
        # time.sleep(0.01)
        write_i32(serial_file, servo_distances[i])
        # time.sleep(0.01)
        wait_for_order(
            serial_file, wait_order=r_order, debug=debug, read_delay=read_delay
        )


def dlength_to_angle(dlength: List[float], debug: bool = False):
    N_groove = 1
    Z_1 = 36
    Z_2 = 13
    U1 = N_groove / Z_1
    U2 = Z_1 / Z_2
    U_tot = U1 * U2
    d_spole = 7e-3
    dist_to_deg = 180 / (np.pi * U_tot * d_spole)
    motor_adjustment = [1.212577713, 1.234914144, 1.228465357]
    dlength = [d * motor_adjustment[i] for i, d in enumerate(dlength)]
    if debug:
        """print("U1: ", U1)
        print("U2: ", U2)
        print("U_tot: ", U_tot)
        print("dist_to_deg: ", dist_to_deg)"""
        print("dlength: ", dlength)
        print("dang", [int(d * dist_to_deg) for d in dlength])
    return [int(d * dist_to_deg) for d in dlength]


def connect_to_arduino(serial_port=None):
    try:
        serial_file = open_serial_port(
            serial_port=serial_port, baudrate=115200, timeout=None
        )
    except Exception as e:
        raise e

    # serial_file = open_serial_port(baudrate=115200, timeout=None)

    is_connected = False
    # Initialize communication with Arduino
    i = 1
    while not is_connected:
        print("Waiting for arduino...")
        print("i: ", i)
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            write_order(serial_file, Order.ALREADY_CONNECTED)
            is_connected = True
    return serial_file


if __name__ == "__main__":
    try:
        serial_file = open_serial_port(baudrate=115200, timeout=None)
    except Exception as e:
        raise e

    # serial_file = open_serial_port(baudrate=115200, timeout=None)

    is_connected = False
    # Initialize communication with Arduino
    i = 1
    while not is_connected:
        print("Waiting for arduino...")
        print("i: ", i)
        write_order(serial_file, Order.ALREADY_CONNECTED)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True

    print("Connected to Arduino")
    print("i: ", i)

    positions = [
        [5000, 5000, 5000],
        [-5000, -5000, -5000],
        [500, 500, 500],
        [-500, -500, -500],
        [500, 500, 500],
        [-500, -500, -500],
        [90, 90, 90],
        [180, 180, 180],
        [270, 270, 270],
        [360, 360, 360],
        [5, 5, 5],
        [90, 90, 90],
        [180, 180, 180],
        [270, 270, 270],
        [360, 360, 360],
    ]
    pos_vec = [0, 0, 0]
    ang_vec = [0, 0, 0]
    test_servo = True
    test_sensor = False
    servo_printout = True
    sensor_printout = True
    for vec in positions:
        for element in vec:
            element *= 10
    for j in range(10):
        # order = read_order(serial_file)
        # data = read_i16(serial_file)

        # print("Ordered received: {:?}", order)
        # print("Data: {:?}", data)
        print("i=", j)
        if test_servo:
            print("servo")
            write_servo_orders(
                serial_file,
                positions[j],
                debug=servo_printout,
            )

        time.sleep(1)
        if test_sensor:
            print("Sensor")
            get_sensor_data2(serial_file, pos_vec, ang_vec, debug=sensor_printout)
            time.sleep(0.1)
            # Magnus
