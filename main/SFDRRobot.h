/*
 * Current power adjust is a bit sketchy. All seems to work though.
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */


#include <Servo.h>
#include <math.h>

#define DRIVE_UPDATE_MS 500
#define TURN_POWER 0.2
#define LEFT_SERVO_PIN 31
#define RIGHT_SERVO_PIN 30
#define DRIVING_SERVO_SPEED 13.333
#define DRIVING_SERVO_MS_MIN 1.3
#define DRIVING_SERVO_MS_MAX 1.7
#define WHEEL_RADIUS 0.0325
#define WHEEL_DISTANCE 0.111
#define BALL_X_OFFSET 0
#define BALL_Y_OFFSET 0

#define TURN_ADJUST 1.5//over 1

class SFDRRobot {
  protected:
  double current_x = 0;
  double current_y = 0;
  double current_a = 0;

  Servo left;
  Servo right;

  double current_left_p = 0;
  double current_right_p = 0;

  double x_dest = 2;
  double y_dest = 2;
  char mode_dest = 0;
  //'f' for only go forward
  //'b' for only go backward
  //'g' for get ball
  //'a' for any direction
  //'l' for any direction, local coordinates

  int last_position_update_ms = 0;

  void set_servo(Servo& servo, double p) const;
  void global_to_local(double dx, double dy, double a, double& lx, double& ly) const;
  void local_to_global(double lx, double ly, double a, double& dx, double& dy) const;

  void p_for_lx_ly(double lx, double ly, double& lp, double& rp);

  public:
  virtual void setup();
  void set_left(double p);
  void set_right(double p);
  void set_destination(double x, double y, int mode);

  void set_position(double x, double y, double a);
  void predict_position(double dt, double& new_x, double& new_y, double& new_a) const;
  void update_motors();
  void update_motors_left_turn();
  void update_motors_right_turn();
  void set_destination(double x, double y, char mode);
  void turn_off();

  virtual void update(int ms);

  double get_d() const;
  void report_heading() const;

  double point_distance(double x1, double y1, double x2, double y2) const;
};

void SFDRRobot::setup() {
  left.attach(LEFT_SERVO_PIN);
  right.attach(RIGHT_SERVO_PIN);
}
void SFDRRobot::set_servo(Servo& servo, double p) const {
  //p between -1 and 1
  servo.writeMicroseconds(1000*((DRIVING_SERVO_MS_MAX-DRIVING_SERVO_MS_MIN)*(1+p)/2+DRIVING_SERVO_MS_MIN));
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
  double sr = current_left_p*DRIVING_SERVO_SPEED*dt*WHEEL_RADIUS;
  double sl = current_right_p*DRIVING_SERVO_SPEED*dt*WHEEL_RADIUS;
  double sm = (sr+sl)/2;
  double a = (sr-sl)/WHEEL_DISTANCE;

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

  double sr = sm+WHEEL_DISTANCE*a;
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
void SFDRRobot::update_motors_left_turn(){
  set_left(-TURN_POWER);
  set_right(TURN_POWER);
}
void SFDRRobot::update_motors_right_turn(){
  set_left(TURN_POWER);
  set_right(-TURN_POWER);
}
void SFDRRobot::update_motors() {
  int sign_lx = 1;
  double lx, ly;
  if(mode_dest == 'l') {
    lx = x_dest;
    ly = y_dest;
  } else {
    global_to_local(x_dest-current_x,y_dest-current_y,current_a,lx,ly);
  }
  double r = 0.1/get_d();
  lx = lx*r;
  ly= ly*r;
  if(mode_dest == 'g') {//get ball
    lx += BALL_X_OFFSET;
    ly += BALL_Y_OFFSET;
  }
  if(mode_dest == 'g' || mode_dest == 'f') {//only go forward
    if (lx < 0) {
      if(ly > 0 ) {
        update_motors_left_turn();
      } else {
        update_motors_right_turn();
      }
      return;//turn procedure
    }
  } else if (mode_dest == 'b') {//only go backward
    if (lx > 0) {
      if(ly > 0) {
        update_motors_right_turn();
      } else {
        update_motors_left_turn();
      }
      return;//turn procedure
    } else {
      lx = abs(lx);
      sign_lx = -1;
    }
  } else {//'l' or 'a'
    sign_lx = lx/abs(lx);
    lx = abs(lx);
  }

  double lp, rp;
  p_for_lx_ly(lx,ly,lp,rp);

  lp = lp*sign_lx;
  rp = rp*sign_lx;
  
  if(abs(rp) > abs(lp)) {//power correction
    lp = lp/abs(rp);
    rp = rp/abs(rp);
  } else {
    rp = rp/abs(lp);
    lp = lp/abs(lp);
  }

  //slows down turns, cause they happen too fast for updates
  double mult = 1-pow(abs(abs(lp)-abs(rp)),TURN_ADJUST);
  lp = lp*mult;
  rp = rp*mult;
  
  set_left(lp);
  set_right(rp);
}
void SFDRRobot::report_heading() const {
  double lx, ly;
  global_to_local(x_dest-current_x,y_dest-current_y,current_a,lx,ly);
  double a = atan2(ly,lx);
  Serial.print("SDFR:O");
  if(current_left_p || current_right_p){
    Serial.print("n");
  } else {
    Serial.print("ff");
  }
  Serial.print(" at x = ");
  Serial.print(current_x);
  Serial.print(", y = ");
  Serial.print(current_y);
  Serial.print(", a = ");
  Serial.print(current_a);
  Serial.print(" heading to x = ");
  Serial.print(x_dest);
  Serial.print(", y = ");
  Serial.print(y_dest);
  Serial.print(", in mode = ");
  Serial.print(mode_dest);
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
  return point_distance(current_x,current_y,x_dest,y_dest);
}
double SFDRRobot::point_distance(double x1, double y1, double x2, double y2) const {
  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
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
void SFDRRobot::turn_off(){
  set_left(0);
  set_right(0);
}
