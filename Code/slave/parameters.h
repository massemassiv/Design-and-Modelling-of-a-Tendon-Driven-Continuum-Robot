#ifndef PARAMETERS_H
#define PARAMETERS_H

#define SERIAL_BAUD 115200  // Baudrate
#define MOTOR_PIN 12
#define DIRECTION_PIN 13
#define SERVOMOTOR_PIN 6
#define INITIAL_THETA 110  // Initial angle of the servomotor
// Min and max values for motors
#define THETA_MIN 60
#define THETA_MAX 150
#define SPEED_MAX 100

// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG true
#endif
