#include "config.h"
#include "IRSensor.h"
#include "RadioRobot.h"

#include "Switch.h"

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
  rr.setup();
  Serial.println("Beggining loop");

  attachCoreTimerService(time_loop);//last
}

void loop() {//code intensive, not time sensitive
  //Serial.println("Main loop executing");
  //rr.run();
  Serial.println(digitalRead(6));
}

uint32_t time_loop(uint32_t time) {//time sensitive, low code impact
  int ms = millis();

  sensor_A.update(ms);
  sensor_B.update(ms);

  return (time+CORE_TICK_RATE/100);
}
