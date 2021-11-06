
#include "PIDMotor.h"
#include "IRSensor.h"
#include "Switch.h"

#define SENSOR_A_PIN A7//orange red brown
#define SENSOR_A_C {17.9079728553213,-0.331182109936777,0.00269786098508671,-0.0000122657211239707,0.0000000337137416663478,-0.0000000000570965155981226,0.0000000000000579855812295257,-0.000000000000000032161423789784,0.00000000000000000000741425519352701}
#define ZERO_PIN 39
#define SEE_THRESHOLD 0.75//raw volts for efficiency
#define ZERO_A_INCREMENT 0.10
#define ZERO_ANGLE 5.5//angle
#define ANGLE_RANGE 2.5
#define MAX_OFFSET 2.2

#define A_MIN (ZERO_ANGLE-ANGLE_RANGE-MAX_OFFSET)
#define A_MAX (ZERO_ANGLE-MAX_OFFSET)

#define NUM_ANGLES 15
#define SCAN_DELAY_MS 50

class Radar{
  private:
  IRSensor sensor;
  PIDMotor motor;
  Switch zero;
  public:
  void setup();
  void update(int us);
  void get_local(double a, double d, double& lx, double& ly) const;
  void find_object(bool& found, double& lx, double& ly);
};
void Radar::setup() {
  Serial.println("Setting up radar.");
  double c[NUMBER_OF_COEFFICIENTS] = SENSOR_A_C;
  sensor.setup(SENSOR_A_PIN,c);
  zero.setup(ZERO_PIN);
  int start = millis();
  Serial.println("Zeroing radar");
  double a;
  motor.setup();
  motor.set_angle(-0.3);
  while(!zero.get_val()) {
    a = ZERO_A_INCREMENT*(millis()-start)/1000;
    //Serial.println(a);
    //Serial.print("Calibrating for ");
    //Serial.print(a);
    //Serial.print(" at ");
    //Serial.println(motor.get_angle());
    motor.set_angle(a);
  }
  motor.set_position(ZERO_ANGLE);
  motor.set_angle((A_MIN+A_MAX)/2);
  delay(100);
}

void Radar::update(int us) {
  sensor.update(us);
  motor.update(us);
}
void Radar::get_local(double a, double d, double& lx, double& ly) const {
  lx = cos(a)*d;
  ly = sin(a)*d;
}
void Radar::find_object(bool& found, double& lx, double& ly){
  double a, d, min_a;
  double min_d = 10;
  for(int i = 0; i < NUM_ANGLES; i++) {
    a = A_MIN+ANGLE_RANGE/NUM_ANGLES*i;
    motor.set_angle(a);
    delay(SCAN_DELAY_MS);
    d = sensor.get_d();
    if(d < min_d) {
      min_d = d;
      min_a = a;
    }
  }
  Serial.print("Scan found a lowest d = ");
  Serial.print(min_d);
  Serial.print(" at a = ");
  Serial.print(min_a);
  Serial.print(" and does ");
  if(min_d < SEE_THRESHOLD) {
    found = true;
    get_local(min_a,min_d,lx,ly);
  } else {
    Serial.print("not ");
    found = false;
  }
  Serial.print("recommend persuit at lx =  ");
  Serial.print(lx);
  Serial.print(", ly = ");
  Serial.print(ly);
  motor.set_angle(A_MIN);
}
