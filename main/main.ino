#include "IRSensor.h"
#include "RadioRobot.h"

#include "Switch.h"

RadioRobot rr;

void setup() {
  Serial.begin(115200);

  attachCoreTimerService(time_loop);//last

  Serial.println("Setting up radio robot");
  rr.setup();
  Serial.println("Beggining loop");

  //attachCoreTimerService(time_loop);//last
}

void loop() {//code intensive, not time sensitive
  //Serial.println("Main loop executing");
  rr.run();
}

int counter = 0;//10us
uint32_t time_loop(uint32_t time) {//time sensitive, low code impact
  //int ms = millis();

  rr.update(counter);

  counter+=10;
  return (time+CORE_TICK_RATE/100);
}
