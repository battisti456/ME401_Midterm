
#include "PIDMotor.h"
#include "IRSensor.h"
#include "Switch.h"

#define SENSOR_A_PIN A7//orange red brown
#define SENSOR_A_C {17.9079728553213,-0.331182109936777,0.00269786098508671,-0.0000122657211239707,0.0000000337137416663478,-0.0000000000570965155981226,0.0000000000000579855812295257,-0.000000000000000032161423789784,0.00000000000000000000741425519352701}
#define ZERO_PIN 39
#define ZERO_ANGLE 0//angle
#define SEE_THRESHOLD 0.75//raw volts for efficiency
#define RADAR_UPDATE_MS 100
#define A_MIN 2.914
#define A_MAX 4.2887
#define A_INCREMENT 0.1
#define ZERO_A_INCREMENT 0.01
#define MEMORY_MS 500

class Radar {
  private:
  IRSensor sensor;
  PIDMotor motor;
  Switch zero;

  bool currently_tracking_something = 0;
  int time_first_tracked = 0;
  int time_last_tracked = 0;
  double approx_a = 0;
  double approx_d = 0;

  int currently_turning =1;
  
  double last_a_dir_change = 0;
  int last_css = 0;

  int dir = 1;

  public:
  void setup();
  void update(int ms);
  void update_sense();

  void set_turning(double t);

  void get_local(double& lx, double& ly);
  bool get_tracking() const;
  int get_time() const;
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
  while(1) {
    a = ZERO_A_INCREMENT*(millis()-start)/1000;
    //Serial.println(a);
    Serial.print("Calibrating for ");
    Serial.print(a);
    Serial.print(" at ");
    Serial.println(motor.get_angle());
    motor.set_angle(a);
  }
  motor.set_position(A_MAX);
}
void Radar::update(int ms) {
  sensor.update(ms);
  motor.update(ms);
  if(ms%RADAR_UPDATE_MS == 0) {
    update_sense();
  }
}
void Radar::get_local(double& lx, double& ly){
  double a = motor.get_angle();
  double d = sensor.get_d();
  lx = cos(a)*d;
  ly = sin(a)*d;
}
void Radar::update_sense() {
  if(sensor.get_d() > SEE_THRESHOLD) {
    if(!currently_tracking_something) {
      currently_tracking_something = true;
    }
    time_last_tracked = millis();
    dir = 0;
  } else if (currently_tracking_something) {
    /*if(millis() - time_last_tracked > MEMORY_MS) {
      currently_tracking_something = false;
    } else {
      dir = -currently_turning;
    }*/
  }
  
  double a =  motor.get_angle() + dir* A_INCREMENT;
  if(a > A_MAX) {
    a = A_MAX;
    dir = -1;
  } else if (a < A_MIN) {
    a = A_MIN;
    dir = 1;
  }
  motor.set_angle(a);
}
void Radar::set_turning(double t){
  currently_turning = t;
}
bool Radar::get_tracking() const {
  return currently_tracking_something;
}
int Radar::get_time() const{
  return time_last_tracked-time_first_tracked;
}
