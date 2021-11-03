/*Healing behavior works.
 * 
 * 
 * 
 * 
 */
#include "SFDRRobot.h"
#include "ME401_Radio.h"
#include "Radar.h"
#include "Switch.h"

#define TEAM_ID 22

#define NUM_HEALTHY_RAYS 20
#define HEALTHY_MOVEMENT_DISTANCE 0.2
#define BALL_RAYS 3

#define TL_PIN 6
#define TR_PIN 7
#define BL_PIN 8
#define BR_PIN 9

#define CORRECTION_TIME_MS 100
#define CORRECTION_POWER 0.25

#define COLLECTOR_POWER 0.25

#define BALL_IN_CORNER_DETECT_THRESHOLD 0.025                                                                                                                                                                                                                                                                                        







#define COLLECTOR_SERVO_PIN 32

#define SENSOR_B_PIN A8//blue green yellow
#define SENSOR_B_C {5.49976232471757,-0.0742105466012858,0.000459695958580232,-0.00000159661150092819,0.00000000328284524668939,-0.0000000000039055365092165,0.00000000000000235526000053281,-0.000000000000000000371191021601162,-0.000000000000000000000163477511119758}

enum StateMachineState {
  HEALTHY = 0,
  ZOMBIE = 1,
  HEALING = 2,
  INVALID = 3
};

class RadioRobot: public SFDRRobot {
  private:
  RobotPose current_pose;
  StateMachineState current_state = HEALTHY;
  //Radar radar;

  Switch tl_coll, tr_coll, bl_coll, br_coll;

  Servo collector;

  IRSensor sensor;

  Radar radar;
  public:
  void setup();
  void update(int ms);
  void run();

  void update_state();
  void update_behavior();

  void healthy_behavior();
  void healing_behavior();
  void zombie_behavior();

  void collision_behavior();

  void find_closest_ball(double x, double y, double& min_d, int& min_i) const;
  void find_closest_zombie(double x, double y, double& min_d, int& min_i) const;

  double con(int val) const;

  void set_collector(double p);
};

void RadioRobot::set_collector(double p) {
  set_servo(collector,p);
}

void RadioRobot::setup(){
  radar.setup();
  SFDRRobot::setup();
  setupRadio(TEAM_ID);
  tl_coll.setup(TL_PIN);
  tr_coll.setup(TR_PIN);
  bl_coll.setup(BL_PIN);
  br_coll.setup(BR_PIN);
  collector.attach(COLLECTOR_SERVO_PIN);
  double c[NUMBER_OF_COEFFICIENTS] = SENSOR_B_C;
  sensor.setup(SENSOR_B_PIN,c);
}
void RadioRobot::run() {
  //update radio info
  updateRobotPoseAndBallPositions();
  current_pose = getRobotPose(TEAM_ID);
  printRobotPose(current_pose);

  //reverse from collisions. couldn't get interrupt to work, but that wasn't mentioned in tthe requirements
  collision_behavior();
  //update_state();

  //update position info
  if (current_state == ZOMBIE) {//cannot use radio posiitoning
    predict_position(con(last_position_update_ms-millis()),current_x,current_y,current_a);
    last_position_update_ms = millis();
  } else {//can use radio positioning
    set_position(con(current_pose.x),con(current_pose.y), con(current_pose.theta));
  }
  
  //update intented behavior
  update_behavior();
  
  //update motor speeds
  update_motors();

  //tell radar which direction robot is truning
  if(abs(current_left_p) > abs(current_right_p)) {
    radar.set_turning(current_left_p/abs(current_left_p));
  } else {
    radar.set_turning(-1*current_right_p/abs(current_right_p));
  }

  report_heading();

}
void RadioRobot::collision_behavior() {
  int tl,tr,bl,br;
  tl = tl_coll.get_val();
  tr = tr_coll.get_val();
  bl = bl_coll.get_val();
  br = tr_coll.get_val();
  if(tl || tr || bl || br) {
    double pl, pr;
    if(tl) {//using elifs cause we presuppose we are driving and only one correction is needed
      pl = -CORRECTION_POWER/2;
      pr = -CORRECTION_POWER;
    } else if (tr) {
      pl = -CORRECTION_POWER;
      pr = -CORRECTION_POWER/2;
    } else if (bl) {
      pl = CORRECTION_POWER/2;
      pr = CORRECTION_POWER;
    } else if (br) {
      pl = CORRECTION_POWER;
      pr = CORRECTION_POWER/2;
    }
    set_left(pl);
    set_right(pr);
    delay(CORRECTION_TIME_MS);
  }
}
void RadioRobot::update_state() {
  current_state = (StateMachineState) (current_pose.zombie + current_pose.healing*2);
}
void RadioRobot::update_behavior() {
  switch(current_state) {
    case HEALTHY://hunting balls while avoiding zombies.
      set_collector(COLLECTOR_POWER);
      healthy_behavior();
    break;
    case ZOMBIE://hunt for healthy
      set_collector(0);
      zombie_behavior();
    break;
    case HEALING://seek out nearest zombie
      set_collector(0);
      healing_behavior();
    default:
      turn_off();
    break;
  }
}
void RadioRobot::healthy_behavior() {
  if(sensor.get_d() < BALL_IN_CORNER_DETECT_THRESHOLD) {
    Serial.println("Ball stuck a bit in corner");
    set_left(0);
    set_right(0.2);
    return;
  }
  double d = 0;
  int index = 0;
  find_closest_ball(current_x,current_y,d,index);
  if(d<HEALTHY_MOVEMENT_DISTANCE) {//go for ball no matter what
    Serial.print("Ball at x = ");
    Serial.print(con(ballPositions[index].x));
    Serial.print(", y = ");
    Serial.print(con(ballPositions[index].y));
    Serial.print(" has run out of time.");
    set_destination(con(ballPositions[index].x),con(ballPositions[index].y),'g');
  } else {//go for one of the BALL_RAYS least dangerous positions out of NUM_HEALTHY_RAYS which is closest to a ball
    double zombie_d[BALL_RAYS] = {0};
    double x_vals[BALL_RAYS] = {0};
    double y_vals[BALL_RAYS] = {0};
  
    double x = 0;
    double y = 0;
    for(int i  = 0; i < NUM_HEALTHY_RAYS; i++) {//finding BALL_RAYS least dangerous positions
      x = current_x+HEALTHY_MOVEMENT_DISTANCE*cos(current_a+2*3.14159/NUM_HEALTHY_RAYS*i);
      y = current_y+HEALTHY_MOVEMENT_DISTANCE*sin(current_a+2*3.14159/NUM_HEALTHY_RAYS*i);
      find_closest_zombie(x,y,d,index);
      for(int j = 0; j < BALL_RAYS; j++) {//find furthest points from zombies
        if(d > zombie_d[i]) {
          for(int k = BALL_RAYS -2; k >= i; k--) {
            zombie_d[k+1] = zombie_d[k];
            x_vals[k+1] = x_vals[k];
            y_vals[k+1] = y_vals[k];
          }
          x_vals[i] = x;
          y_vals[i] = y;
          zombie_d[i] = d;
        }
      }
    }
  
    double min_d = 10;//finding ray with closest ball
    for(int i = 0; i < BALL_RAYS; i++) {
      find_closest_ball(x_vals[i],y_vals[i],d,index);
      if( d < min_d) {
        min_d = d;
        x = x_vals[i];
        y = y_vals[i];
      }
    }

    Serial.print("Instead of going to the point of highest safety, at x = ");
    Serial.print(x_vals[0]);
    Serial.print(", y = ");
    Serial.print(y_vals[0]);
    Serial.print(", I am instead going to x = ");
    Serial.print(x);
    Serial.print(", y = ");
    Serial.print(y);
    Serial.print(" just for funzees.");
    set_destination(x,y,'a');
  }
}
void RadioRobot::zombie_behavior() {
  if(radar.get_tracking()) {
    Serial.print("On the trail for ");
    Serial.print(con(radar.get_time())/60/60/24);
    Serial.print(" days.");
    double lx,ly;
    radar.get_local(lx,ly);
    set_destination(lx,ly,'l');
  } else {
    Serial.print("Off awanderin.");
    set_destination(con(random(2000)-1000),con(random(2000)-1000),'l');
  }
}
void RadioRobot::healing_behavior() {//need to test, but on board set to heal and see if it follows nearest zombie on a stick
  double d;
  int i;
  find_closest_zombie(current_x,current_y,d,i);
  if(i == -1) {//no zombies
    turn_off();
  } else {
    set_destination(con(robotPoses[i].x),con(robotPoses[i].y),'a');
    Serial.print("Hunting zombie with ID# ");
    Serial.println(robotPoses[i].ID);
  }
}

void RadioRobot::find_closest_ball(double x, double y, double& min_d, int& min_i) const {
  min_d = 100;
  min_i = -1;
  double d= 0;
  for(int i = 0; i < numBalls; i++) {
    d = point_distance(x,y,con(ballPositions[i].x),con(ballPositions[i].y));
    if(d < min_d) {
      min_d = d;
      min_i = i; 
    }
  }
}
void RadioRobot::find_closest_zombie(double x, double y, double& min_d, int& min_i) const{
  min_d = 100;
  min_i = -1;
  double d = 0;
  for(int i = 0; i < NUM_ROBOTS; i++) {
    if(robotPoses[i].zombie && robotPoses[i].ID != TEAM_ID && robotPoses[i].valid) {//if zombie and not me (just in case wierdness)
      printRobotPose(robotPoses[i]);
      d = point_distance(x,y,con(robotPoses[i].x),con(robotPoses[i].y));
      Serial.println(i);
      Serial.println(d);
      if (d < min_d) {
        min_d = d;
        min_i = i;
      }
    }
  }
}
void RadioRobot::update(int ms) {
  radar.update(ms);
  sensor.update(ms);
}
double RadioRobot::con(int val) const {
  return (double) val/1000;
}
