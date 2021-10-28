#include <Servo.h>
#include <math.h>
#include "config.h"

class SFDRRobot {
  private:
  double current_x = 0;
  double current_y = 0;
  double current_a = 0;

  Servo left;
  Servo right;

  int left_pin = 0;
  int right_pin = 0;

  double servo_ms_min = 0;
  double servo_ms_max = 0;

  double servo_speed = 0;//rad/s at max
  double wheel_radius = 0;//m
  double wheel_distance = 0;//m

  double current_left_p = 0;
  double current_right_p = 0;

  double x_dest = 2;
  double y_dest = 2;
  char mode_dest = 0;

  double ball_x_offset = 0;
  double ball_y_offset = 0;

  int last_position_update_ms = 0;

  void set_servo(Servo& servo, double p) const;
  void global_to_local(double dx, double dy, double a, double& lx, double& ly) const;
  void local_to_global(double lx, double ly, double a, double& dx, double& dy) const;

  void p_for_lx_ly(double lx, double ly, double& lp, double& rp);

  public:
  void setup(int l_pin, int r_pin, double ms_min, double ms_max, double sp, double r, double d);
  void set_left(double p);
  void set_right(double p);
  void set_destination(double x, double y, int mode);

  void set_position(double x, double y, double a);
  void predict_position(double dt, double& new_x, double& new_y, double& new_a) const;
  void update_motors();
  void set_destination(double x, double y, char mode);

  void update(int ms);

  double get_d() const;
  void report_heading() const;
};

void SFDRRobot::setup(int l_pin, int r_pin, double ms_min, double ms_max, double sp, double r, double d) {
  left_pin = l_pin;
  right_pin = r_pin;
  servo_ms_min = ms_min;
  servo_ms_max = ms_max;
  servo_speed = sp;
  wheel_radius = r;
  wheel_distance= d;
  left.attach(left_pin);
  right.attach(right_pin);
}
void SFDRRobot::set_servo(Servo& servo, double p) const {
  //p between -1 and 1
  servo.writeMicroseconds(1000*((servo_ms_max-servo_ms_min)*(1+p)/2+servo_ms_min));
  Serial.println(1000*((servo_ms_max-servo_ms_min)*(1+p)/2+servo_ms_min));
}
void SFDRRobot::set_left(double p) {
  set_servo(left,p);
  current_left_p = p;
}
void SFDRRobot::set_right(double p) {
  set_servo(right,-p);
  current_right_p = p;
}
void SFDRRobot::set_position(double x, double y, double a) {
  current_x = x;
  current_y = y;
  current_a = a;
  last_position_update_ms = millis();
}
void SFDRRobot::predict_position(double dt, double& new_x, double& new_y, double& new_a) const {//only works in deafault driving mode
  double sr = current_left_p*servo_speed*dt*wheel_radius;
  double sl = current_right_p*servo_speed*dt*wheel_radius;
  double sm = (sr+sl)/2;
  double a = (sr-sl)/wheel_distance;

  double dx = sm*cos(a);
  double dy = sm*sin(a);

  new_x = current_x + dx;
  new_y = current_y + dy;
  new_a = current_a + a;
}
void SFDRRobot::update(int ms){
  if (ms%DRIVE_UPDATE_MS == 0) {
    //predict_position((double) (last_position_update_ms-ms)/1000,current_x,current_y,current_a);
    //last_position_update_ms = ms;
    update_motors();
    report_heading();
  }
}
void SFDRRobot::global_to_local(double dx, double dy, double a, double& lx, double& ly) const {
  lx = cos(a)*dx+sin(a)*dy;
  ly = -sin(a)*dx+cos(a)*dy;
}
void SFDRRobot::local_to_global(double lx, double ly, double a, double& dx, double& dy) const {
  global_to_local(lx,ly,-a,dx,dy);
}
void SFDRRobot::p_for_lx_ly(double lx, double ly, double& lp, double& rp) {
  double a = atan2(ly,lx);
  double smx = lx/cos(a);
  double smy = ly/sin(a);
  double sm = (smx+smy)/2;//so no direction preference

  double sr = sm+wheel_distance*a;
  double sl = 2*sm-sr;

  //might give invalid p values, just hope we choose lx ly well
  //Serial.println(1/wheel_radius/servo_speed/DRIVE_UPDATE_MS*1000);//4.62
  lp = sl;
  rp = sr;
}
void SFDRRobot::set_destination(double x, double y, char mode){
  x_dest = x;
  y_dest = y;
  mode_dest = mode;
}
void SFDRRobot::update_motors() {
  double lx, ly;
  global_to_local(x_dest-current_x,y_dest-current_y,current_a,lx,ly);
  double r = 0.1/get_d();
  lx = lx*r;
  ly= ly*r;
  if(mode_dest == 'g') {//get ball
    lx += ball_x_offset;
    ly += ball_y_offset;
  }
  double lp, rp;
  
  int sign_lx = lx/abs(lx);
  lx = abs(lx);

  p_for_lx_ly(lx,ly,lp,rp);

  lp = lp*sign_lx;
  rp = rp*sign_lx;
  
  if(abs(rp) > abs(lp)) {//make sure possible, can only slow it down
    lp = lp/abs(rp);
    rp = rp/abs(rp);
  } else {
    rp = rp/abs(lp);
    lp = lp/abs(lp);
  }
  set_left(lp);
  set_right(rp);
}
void SFDRRobot::report_heading() const {
  double lx, ly;
  global_to_local(x_dest-current_x,y_dest-current_y,current_a,lx,ly);
  double a = atan2(ly,lx);
  Serial.print("SDFR:At x = ");
  Serial.print(current_x);
  Serial.print(", y = ");
  Serial.print(current_y);
  Serial.print(", a = ");
  Serial.print(current_a);
  Serial.print(" heading to x = ");
  Serial.print(x_dest);
  Serial.print(", y = ");
  Serial.print(y_dest);
  Serial.print(" (at lx = ");
  Serial.print(lx);
  Serial.print(", ly = ");
  Serial.print(ly);
  Serial.print(", a = ");
  Serial.print(a);
  Serial.print(") with pl = ");
  Serial.print(current_left_p);
  Serial.print(", pr = ");
  Serial.println(current_right_p);
}
double SFDRRobot::get_d() const{
  return sqrt(pow(x_dest-current_x,2)+pow(y_dest-current_y,2));
}
void test_SFDRRobot(SFDRRobot& driving_robot) {
  double d = driving_robot.get_d();
  if(d < 0.1) {
    double x = (double) random(2000)/1000;
    double y = (double) random(2000)/1000;
    driving_robot.set_destination(x,y,'d');
    Serial.print("Setting random destination at (");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println(")");
  }
}
