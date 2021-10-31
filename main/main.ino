#include "config.h"
#include "IRSensor.h"
#include "PIDMotor.h"
#include "RadioRobot.h"

RadioRobot rr;
IRSensor sensor_A;
IRSensor sensor_B;

void setup() {
  Serial.begin(115200);

  Serial.println("Setting up sensors");
  double Ac[NUMBER_OF_COEFFICIENTS] = SENSOR_A_C;
  sensor_A.setup(SENSOR_A_PIN,Ac);
  double Bc[NUMBER_OF_COEFFICIENTS] = SENSOR_A_C;
  sensor_B.setup(SENSOR_B_PIN,Bc);

  Serial.println("Setting up radio robot");
  rr.setup(LEFT_SERVO_PIN,RIGHT_SERVO_PIN,DRIVING_SERVO_MS_MIN,DRIVING_SERVO_MS_MAX,DRIVING_SERVO_SPEED,WHEEL_RADIUS,WHEEL_DISTANCE);
  Serial.println("Beggining loop");


  attachCoreTimerService(time_loop);//last
}

void loop() {
  Serial.println("Main loop executing");
  rr.run();
}

uint32_t time_loop(uint32_t time) {
  int ms = millis();

  sensor_A.update(ms);
  sensor_B.update(ms);

  return (time+CORE_TICK_RATE/100);
}
