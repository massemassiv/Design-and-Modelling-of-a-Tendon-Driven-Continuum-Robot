#ifndef ORDER_H
#define ORDER_H

// Define the orders that can be sent and received

enum Order{
    HELLO = 0,
    SERVO = 1,
    MOTOR = 2,
    ALREADY_CONNECTED = 3,
    ERROR = 4,
    RECEIVED = 5,
    STOP = 6,
    
    GET_SENSOR = 7,
    SENT_POS_X = 8,
    SENT_POS_Y = 9,
    SENT_POS_Z = 10,
    SENT_ANGLE_X = 11,
    SENT_ANGLE_Y = 12,
    SENT_ANGLE_Z = 13,

    SENT_SERVO1=14,
    SENT_SERVO2=15,
    SENT_SERVO3=16,

    RECEIVED_SERVO1=17,
    RECEIVED_SERVO2=18,
    RECEIVED_SERVO3=19,
};
typedef enum Order Order;
#endif
