#include <Servo.h>
#include <math.h>

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
