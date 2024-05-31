import time
from typing import List

from arduino_kommunikation import (
    connect_to_arduino,
    read_i16,
    wait_for_order,
    write_order,
)
from egna_orders import Order


def get_sensor_data(
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
        time.sleep(0.01)

    for i, w_order in enumerate(angle_order):
        # order = read_order(serial_file)
        wait_for_order(serial_file, w_order, debug=debug)
        angle_vec[i] = read_i16(serial_file)
        write_order(serial_file, Order.RECEIVED)
        time.sleep(0.01)
    if debug:

        print("position")
        # print(f"ORDER{i}: ", order)
        print("AngX", angle_vec[0])
        print("AngY", angle_vec[1])
        print("AngZ", angle_vec[2])


if __name__ == "__main__":
    serial_file = connect_to_arduino()
    debug = True
    for i in range(100):
        position_vec = [0, 0, 0]
        angle_vec = [0, 0, 0]
        get_sensor_data(serial_file, position_vec, angle_vec, debug)
        print("Position: \n", position_vec, "Angle:\n", angle_vec)
