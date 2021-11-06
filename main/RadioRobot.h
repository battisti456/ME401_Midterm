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

#define NUM_HEALTHY_RAYS 30
#define HEALTHY_MOVEMENT_DISTANCE 0.2
#define BALL_RAYS 10

#define TL_PIN 6
#define TR_PIN 7
#define BL_PIN 8
#define BR_PIN 9

#define CORRECTION_TIME_US 1000000
#define CORRECTION_POWER 0.25

#define COLLECTOR_POWER 0.25

#define BALL_IN_CORNER_DETECT_THRESHOLD 0.025                                                                                                                                                                                                                                                                                        

#define ZOMBIE_RUN_MS 2000

#define COLLECTOR_SERVO_PIN 32

#define SENSOR_B_PIN A8//blue green yellow
#define SENSOR_B_C {5.49976232471757,-0.0742105466012858,0.000459695958580232,-0.00000159661150092819,0.00000000328284524668939,-0.0000000000039055365092165,0.00000000000000235526000053281,-0.000000000000000000371191021601162,-0.000000000000000000000163477511119758}

#define N_DIV 3

enum StateMachineState {
  HEALTHY = 0,
  ZOMBIE = 1,
  HEALING = 2,
  INVALID = 3
};

class RadioRobot: public SFDRRobot {
  private:
  RobotPose current_pose;
  StateMachineState current_state = ZOMBIE;

  Switch tl_coll, tr_coll, bl_coll, br_coll;

  Servo collector;

  IRSensor sensor;

  Radar radar;

  public:
  void setup();
  void update(int us);
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
  Serial.print("Current pose: ");
  printRobotPose(current_pose);

  //reverse from collisions. couldn't get interrupt to work, but that wasn't mentioned in tthe requirements
  Serial.println("Applying collision behavior");
  collision_behavior();
  update_state();

  //update position info
  Serial.print("Update position using ");
  if (current_state == ZOMBIE) {//cannot use radio posiitoning
    Serial.println("best guess");
    predict_position(con(last_position_update_ms-millis()),current_x,current_y,current_a);
    last_position_update_ms = millis();
  } else {//can use radio positioning
    Serial.println("radio information");
    set_position(con(current_pose.x),con(current_pose.y), con(current_pose.theta));
  }
  
  //update intented behavior
  Serial.print("Do behavior according to state: ");
  update_behavior();
  
  //update motor speeds
  Serial.println("Calculate and set motor speeds");
  update_motors();
  
  report_heading();

}
void RadioRobot::collision_behavior() {
  int tl,tr,bl,br;
  tl = tl_coll.get_val();
  tr = tr_coll.get_val();
  bl = bl_coll.get_val();
  br = br_coll.get_val();
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
    Serial.print("Correcting a collision with pl = ");
    Serial.print(pl);
    Serial.print(", pr = ");
    Serial.println(pr);
    set_left(pl);
    set_right(pr);
    delay(CORRECTION_TIME_US/1000);
  }
}
void RadioRobot::update_state() {
  current_state = (StateMachineState) (current_pose.zombie + current_pose.healing*2);
}
void RadioRobot::update_behavior() {
  switch(current_state) {
    case HEALTHY://hunting balls while avoiding zombies.
      Serial.println("healthy");
      set_collector(-COLLECTOR_POWER);
      healthy_behavior();
    break;
    case ZOMBIE://hunt for healthy
      Serial.println("zombie");
      set_collector(0);
      zombie_behavior();
    break;
    case HEALING://seek out nearest zombie
    Serial.println("healing");
      set_collector(0);
      healing_behavior();
    default:
      turn_off();
    break;
  }
}
void RadioRobot::healthy_behavior() {
  static bool in_collect_ball_mode = false;
  static double last_ball_x = 0;
  static double last_ball_y = 0;
  double ball_dist = sensor.get_d();
  if(ball_dist < BALL_IN_CORNER_DETECT_THRESHOLD) {
    Serial.print("Ball stuck a bit in corner at d = ");
    Serial.println(ball_dist);
    set_left(0);
    set_right(0.2);
    return;
  }
  double ball_d, zombie_d;
  int ball_i, zombie_i;

  find_closest_ball(current_x,current_y,ball_d,ball_i);
  find_closest_zombie(current_x,current_y,zombie_d,zombie_i);

  double ball_x = con(ballPositions[ball_i].x);
  double ball_y = con(ballPositions[ball_i].y);
  bool ignore_ball = point_distance(ball_x,ball_y,last_ball_x,last_ball_y) > 0.05;
  last_ball_x = ball_x;
  last_ball_y = ball_y;

  int val = 2*(zombie_d == 100) + (ball_d == 100);
  switch(val) {
    case 0:
    Serial.println("There are zombies and there are balls.");
    break;
    case 1:
    Serial.println("There are zombies but there are no balls.");
    break;
    case 2:
    Serial.println("There are no zombies but there are balls.");
    break;
    case 3:
    Serial.println("There are no zombies and there are no balls.");
  }
  if(val == 3) {//nothing to do
    turn_off();
    return;
  } else if(val == 2) {
    in_collect_ball_mode = true;
  } else if (ball_d < 0.3) {
    in_collect_ball_mode = true;
  } else if (zombie_d - ball_d > 0.3) {
    in_collect_ball_mode = true;
  } else if (ball_d > 0.5) {
    in_collect_ball_mode = false;
  } else if (zombie_d - ball_d < -0.3) {
    in_collect_ball_mode = false;
  }
  
  /*
  else if(val == 2 || (val == 0 && (ball_d < 0.3 || zombie_d > ball_d))) {//go after ball
    set_destination(con(ballPositions[ball_i].x),con(ballPositions[ball_i].y),'f',false,true);
  } else {//run from zombie
    double x,y,dx,dy;
    dx = con(robotPoses[zombie_i].x)-current_x;
    dy = con(robotPoses[zombie_i].y)-current_y;
    x = current_x - dx;
    y = current_y - dy;
    set_destination(x,y,'a');
  }*/
  if(in_collect_ball_mode) {
    if(!ignore_ball) {
      set_destination(con(ballPositions[ball_i].x),con(ballPositions[ball_i].y),'f',false,true);
    } else {
      Serial.println("Ignoring ball.");
    }
  } else {
    double x,y,dx,dy;
    dx = con(robotPoses[zombie_i].x)-current_x;
    dy = con(robotPoses[zombie_i].y)-current_y;
    x = current_x - dx;
    y = current_y - dy;
    set_destination(x,y,'a');
  }
}
void RadioRobot::zombie_behavior() {
  static int last_t = 0;
  Serial.println("Zombie behavior");
  int t= millis();
  if(t-last_t > ZOMBIE_RUN_MS) {//so scan is not done too often
    Serial.println("heyyyyyyyy");
    last_t = t;
    
    turn_off();//during scan
    bool found;
    double lx, ly;
    Serial.println("Scanning for object");
    radar.find_object(found,lx,ly);
    if(found) {
      set_destination(lx,ly,'a',true,false);
    } else {
      set_destination(con(random(2000)-1000),con(random(2000)-1000),'a',true,false);
    }
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
      //printRobotPose(robotPoses[i]);
      d = point_distance(x,y,con(robotPoses[i].x),con(robotPoses[i].y));
      if (d < min_d) {
        min_d = d;
        min_i = i;
      }
    }
  }
}
void RadioRobot::update(int us) {
  radar.update(us);
  sensor.update(us);
}
double RadioRobot::con(int val) const {
  return (double) val/1000;
}
