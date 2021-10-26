#ifndef CONFIG
#define CONFIG

//driving
#define LEFT_SERVO_PIN 31
#define RIGHT_SERVO_PIN 30
#define DRIVING_SERVO_SPEED 13.333
#define DRIVING_SERVO_MS_MIN 1.3
#define DRIVING_SERVO_MS_MAX 1.7
#define WHEEL_RADIUS 0.0325
#define WHEEL_DISTANCE 0.111

#define DRIVE_UPDATE_MS 500

//ir sensors
#define NUMBER_OF_COEFFICIENTS 3
#define FILTER_LENGTH 5

#define SENSOR_A_PIN A7//orange red brown
#define SENSOR_B_PIN A8//blue green yellow

#define IRSENSOR_UPDATE_MS 100

//pid motor
#define PIN_A 2
#define PIN_B 20
#define DIR_PIN 4
#define PWM_PIN 3
#define COUNTS_PER_REVOLUTION 2800
#define KP 1
#define KI 0
#define KD 0

#define PIDMOTOR_POSITION_UPDATE_MS 1
#define PIDMOTOR_PID_UPDATE_MS 100

#endif
