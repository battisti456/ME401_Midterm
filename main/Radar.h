
#include "PIDMotor.h"
#include "IRSensor.h"
#include "Switch.h"

#define SENSOR_A_PIN A7//orange red brown
#define SENSOR_A_C {17.9079728553213,-0.331182109936777,0.00269786098508671,-0.0000122657211239707,0.0000000337137416663478,-0.0000000000570965155981226,0.0000000000000579855812295257,-0.000000000000000032161423789784,0.00000000000000000000741425519352701}
#define ZERO_PIN 25
#define ZERO_ANGLE 0//angle
#define SEE_THRESHOLD 100//raw volts for efficiency
#define RADAR_UPDATE_MS 100
#define A_MIN 0
#define A_MAX 1
#define A_INCREMENT 0.1

class Radar {
  private:
  IRSensor sensor;
  PIDMotor motor;
  Switch zero;

  int currently_tracking_something = 0;
  double approx_a = 0;
  double approx_d = 0;

  int dir = 1;
  double last_a_dir_change = 0;
  int last_css = 0;

  public:
  void setup();
  void update(int ms);

  void zero_radar();

  void set_angle(double a);

  void get_local(double& lx, double& ly)const;
};

void Radar::setup() {
  double c[NUMBER_OF_COEFFICIENTS] = SENSOR_A_C;
  sensor.setup(SENSOR_A_PIN,c);
  motor.setup();
  //zero.setup(ZERO_PIN,zero_radar);
}
void Radar::zero_radar(){
  motor.set_position(ZERO_ANGLE);
}
void Radar::update(int ms) {
  sensor.update(ms);
  motor.update(ms);
  if(ms%RADAR_UPDATE_MS == 0) {
    int css = sensor.get_i() > SEE_THRESHOLD;
    double angle = motor.get_angle();
    if(css) {
      approx_a = angle;
      approx_d = sensor.get_d();
    }else if(!css && last_css) {
      dir = -dir;
    } else if (angle - A_INCREMENT < A_MIN) {
      dir = 1;
    } else if (angle + A_INCREMENT > A_MAX) {
      dir = -1;
    }
    set_angle(angle + A_INCREMENT*dir);
    last_css = css;
  }
}
void Radar::set_angle(double a){
  if (a < A_MIN) {
    a = A_MIN;
  } else if ( a > A_MAX) {
    a = A_MAX;
  }
  motor.set_angle(a);
}
void Radar::get_local(double& lx, double& ly) const {
  lx = cos(approx_a)*approx_d;
  ly = sin(approx_a)*approx_d;
}
