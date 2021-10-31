#include "SFDRRobot.h"
#include "ME401_Radio.h"
#include "Radar.h"

#define TEAM_ID 22

#define NUM_HEALTHY_RAYS 20
#define HEALTHY_MOVEMENT_DISTANCE 0.2

enum StateMachineState {
  HEALTHY = 0,
  ZOMBIE = 1,
  HEALING = 2,
  INVALID = 3
};

class RadioRobot: public SFDRRobot {
  private:
  RobotPose current_pose;
  StateMachineState current_state = HEALING;
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

  void find_closest_ball(double x, double y, double& min_d, int& min_i) const;
  void find_closest_zombie(double x, double y, double& min_d, int& min_i) const;
};

void RadioRobot::setup(){
  SFDRRobot::setup();
  setupRadio(TEAM_ID);
}
void RadioRobot::run() {
  //update radio info
  updateRobotPoseAndBallPositions();
  current_pose = getRobotPose(TEAM_ID);
  //update_state();
  on = current_pose.valid;

  //update position info
  if (current_state == ZOMBIE) {//cannot use radio posiitoning
    predict_position((double) (last_position_update_ms-millis())/1000,current_x,current_y,current_a);
    last_position_update_ms = millis();
  } else {//can use radio positioning
    set_position(current_pose.x/1000,current_pose.y/1000,current_pose.theta/1000);
  }
  
  //update intented behavior
  update_behavior();
  
  //update motor speeds
  update_motors();
  
  report_heading();

}
void RadioRobot::update_state() {
  current_state = (StateMachineState) (current_pose.zombie + current_pose.healing*2);
}
void RadioRobot::update_behavior() {
  switch(current_state) {
    case HEALTHY://hunting balls while avoiding zombies.
      healthy_behavior();
    break;
    case ZOMBIE://hunt for healthy
      zombie_behavior();
    break;
    case HEALING://seek out nearest zombie
      healing_behavior();
    default:
     on = 0;
    break;
  }
}
void RadioRobot::healthy_behavior() {
  
}
void RadioRobot::zombie_behavior() {
  
}
void RadioRobot::healing_behavior() {//need to ttest, but on board set to heal and see if it follows nearest zombie on a stick
  double d;
  int i;
  find_closest_zombie(current_x,current_y,d,i);
  if(i == -1) {//no zombies
    on = 0;
  } else {
    set_destination(robotPoses[i].x,robotPoses[i].y,'a');
    Serial.print("Hunting zombie with ID# ");
    Serial.println(robotPoses[i].ID);
  }
}

void RadioRobot::find_closest_ball(double x, double y, double& min_d, int& min_i) const {
  min_d = 100;
  min_i = -1;
  double d= 0;
  for(int i = 0; i < numBalls; i++) {
    d = point_distance(x,y,ballPositions[i].x/1000,ballPositions[i].y/1000);
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
  for(int i = 0; i < numRobots; i++) {
    printRobotPose(robotPoses[i]);
    if(robotPoses[i].zombie && robotPoses[i].ID != TEAM_ID) {//if zombie and not me (just in case wierdness)
      d = point_distance(x,y,robotPoses[i].x/1000,robotPoses[i].y/1000);
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
}
