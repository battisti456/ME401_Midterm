#include "ME401_Radio.h"
#include "movement.h"

enum StateMachineState {
  HEALTHY = 0,
  ZOMBIE = 1,
  HEALING = 2,
  INVALID = 3,
  GET_BALL = 4
};

StateMachineState state = ZOMBIE;
int team_id = 22;

double x_conv = 0.001;//m per int
double y_conv = 0.001;//m per int
double a_conv = 0.001;//radians per int

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up radio.");
  setupRadio(team_id);
  Serial.println("Setting up driving");
  setup_driving();
  Serial.println("Beggining loop");
}

void loop() {
  Serial.println("Update Robot Ball Positions");
  updateRobotPoseAndBallPositions();
  Serial.println("Get State");
  state = get_state();
  
  RobotPose current_pose = getRobotPose(team_id);
  //printRobotPose(current_pose);
  set_current_pos(x_conv*current_pose.x,y_conv*current_pose.y,a_conv*current_pose.theta);
  change_speed_for_x_y(0.2,0.2);
  update_speeds();
  
  delay(100);
}

StateMachineState get_state() {
  RobotPose current_pose = getRobotPose(team_id);
  int n = current_pose.zombie + current_pose.healing*2;
  if(n == 0) {
    if(should_we_get_ball()){
      n = 4;
    }
  }
  Serial.print("State: ");
  Serial.println(n);
  return (StateMachineState) n;
}

int should_we_get_ball() {
  return 0;
}
