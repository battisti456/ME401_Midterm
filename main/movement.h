#include <Servo.h>
#include <math.h>

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
  double expected_run_time = 0;//s, time run at a speed
  double max_angle = 0;
  double turn_distance = 0;


  double current_left_p = 0;
  double current_right_p = 0;

  void set_servo(Servo& servo, double p) const {}
  void global_to_local(double dx, double dy, double a, double& lx, double& ly) const {}
  void local_to_global(double lx, double ly, double a, double& dx, double& dy) const {}

  void p_for_lx_ly(double lx, double ly, double& lp, double& rp){}

  public:
  SFDRRobot(int l_pin, int r_pin, double ms_min, double ms_max, double sp, double r, double d, double max_a, double turn_d) {}

  void setup(){}
  void set_left(double p){}
  void set_right(double p){}

  void update_position(double x, double y, double a) {}
  void predict_position(double dt, double& new_x, double& new_y, double& new_a) const {}
  void go_position(double x, double y) {}
}


SFDRRobot::SFDRRobot(int l_pin, int r_pin, double ms_min, double ms_max, double sp, double r, double d, double t, double max_a, double turn_d) {
  left_pin = l_pin;
  right_pin = r_pin;
  servo_ms_min = ms_min;
  servo_ms_max = ms_max;
  servo_speed = sp;
  wheel_radius = r;
  wheel_distance= d;
  expected_run_time = t;
  max_angle = max_a;
  turn_distance = turn_d;
}
void SFDRRobot::setup() {
  left.attach(left_servo_pin);
  right.attach(right_servo_pin);
}
void SFDRRobot::set_servo(Servo& servo, double p) const {
  //p between -1 and 1
  servo.writeMicroseconds(1000*((servo_ms_max-servo_ms_min)*(1+s)+servo_ms_min));
}
void SFDRRobot::set_left(double p) {
  set_servo(left,p);
  current_left_p = p;
}
void SFDRRobot::set_right(double p) {
  set_servo(right,p);
  current_right_p = p;
}
void SFDRRobot::update_position(double x, double y, double a) {
  current_x = x;
  current_y = y;
  current_a = a;
}
void SFDRRobot::predict_position(double dt, double& new_x, double& new_y, double& new_a) const {
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
void SFDRRobot::global_to_local(double dx, double dy, double a, double& lx, double& ly) const {
  lx = cos(a)*dx+sin(a)*dy;
  ly = -sin(a)*x+cos(a)*dy;
}
void SFDRRobot::local_to_global(double lx, double ly, double a, double& dx, double& dy) const {
  global_to_local(lx,ly,-a,dx,dy);
}
void SFDRRobot::p_for_lx_ly(double lx, double ly, double& lp, double& rp) {
  double a = atan2(ly,lx);
  double smx = dx/cos(a);
  double smy = dy/sin(a);
  double sm = (smx+smy)/2;//so no direction preference

  double sr = sm+wheel_distance*(a-current.a);
  double sl = 2*sm-sr;

  //might give invalid p values, just hope we choose lx ly well
  lp = sl/wheel_radius/servo_speed/expected_run_time;
  rp = sr/wheel_radius/servo_speed/expected_run_time;
}
void SFDRRobot::go_position(double x, double y) {
  double lx, ly;
  global_to_local(x-current_x,y-current_y,current_a,lx,ly);
  a = atan2(ly,lx);
  if(abs(a) > max_angle) {//choose new lx, ly
    double used_angle = a/abs(a)*9/10*max_angle;
    lx = cos(used_angle)*turn_distance;
    ly = sin(used_angle)*turn_distance;
  }
  double lp, rp;
  p_for_lx_ly(lx,ly,lp,rp);
  if(abs(rp) > 1 && rp > lp) {//make sure possible, can only slow it down, so makes over expected time
    lp = lp/abs(rp);
    rp = rp/abs(rp);
  } else if (abs(lp) > 1) {
    rp = rp/abs(lp);
    lp = lp/abs(lp);
  }
  set_left(lp);
  set_right(rp);
}


struct pos {
    double x;//in meters
    double y;//in meters
    double a;//in radians
};

void set_servo_power(Servo servo, int s) {
  servo.writeMicroseconds(1500+2*s);
}

int left_servo_pin = 31;
int right_servo_pin = 30;

Servo left;
Servo right;

void set_left(int sp) {
  set_servo_power(left,sp);
}

void set_right(int sp) {
  set_servo_power(right,sp);
}

void setup_driving() {
  left.attach(left_servo_pin);
  right.attach(right_servo_pin);
  set_left(0);
  set_right(0);
}

struct pos current;

int l_speed = 0;//-100 to 100
int r_speed = 0;//-100 to 100

void update_speeds() {
  Serial.print("Setting speed: ");
  Serial.print(l_speed);
  Serial.print(" ");
  Serial.println(r_speed);
  set_left(l_speed);
  set_right(-r_speed);
}

double wheel_distance = 0.111;//measure, in meters
double wheel_circumfrance = 0.067;//measure, in meters
//double speed_mult = 0.00318;
double speed_mult = 0.05;

double a_max = 0.15;
double a_step = 0.1;

void find_sr_sl_for_move(double dx, double dy, double a, double& sr, double& sl) {//SFDR
    double smx = dx/cos(a);
    double smy = dy/sin(a);
    double sm = (smx+smy)/2;//so no direction preference

    sr = sm+wheel_distance*(a-current.a);
    sl = 2*sm-sr;
}

void change_speed_for_x_y(double x1, double y1) {
  double dx = x1 - current.x;
  double dy = y1 - current.y;
  double a = atan(dy/dx);
  

  Serial.print("Speed for dx, dy, a: ");
  Serial.print(dx);
  Serial.print(", ");
  Serial.print(dy);
  Serial.print(", ");
  Serial.println(a);

  if(abs(a) > a_max) {//need to actually figure out!!!!!!!!
    double a_used = a/abs(a)*(a_max-0.1);
    double new_x1 = current.x + a_step*cos(a_used);
    double new_y1 = current.y + a_step*sin(a_used);
    change_speed_for_x_y(new_x1,new_y1);
    return;
  }

  double sr = 0;
  double sl = 0;

  find_sr_sl_for_move(dx,dy,a,sr,sl);

  if(abs(sr) > abs(sl)) {
    sr = sr/abs(sr);
    sl = sl/abs(sl);
  } else {
    sr = sr/abs(sl);
    sl = sl/abs(sl);
  }
  

  l_speed = sr*100*speed_mult;
  r_speed = sl*100*speed_mult;

  
}

void set_current_pos(double x, double y, double a) {
  Serial.print("Setting position to x, y, a: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(a);
  
  current.x = x;
  current.y = y;
  current.a = a;
}
