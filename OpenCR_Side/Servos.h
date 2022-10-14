#include <Dynamixel2Arduino.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#define DXL_DIR_PIN 84
#define baserotator 0
#define basearm 0
#define forearm 0
#define hand 0
#define gripperOpener 0
#define gripperTurner 0 
#define frontFlipper 0
#define backFlipper 0

Dynamixel2Arduino dxl(Serial3,DXL_DIR_PIN);
bool extended = false;

void SetupServo(){
  dxl.begin(57600);
}

float constrain(pos, min, max){
  if (pos < min){
    pos = min;
  }
  if (pos > max){
    pos = max;
  }
  return pos;
}

void ControlDynamixels(sensor_msgs::JointState joint_states_claw, float position_frontFlipper, float position_backFlipper){
  for (i = 0; i < joint_states_claw.length; i++) {
    switch(joint_states_claw.name[i]){
      case "joint0":
        dxl.setGoalPosition(baserotator, joint_states_claw.position[i], UNIT_DEGREE);
        break;
      case "joint1":
        dxl.setGoalPosition(basearm, joint_states_claw.position[i], UNIT_DEGREE);
        break;
      case "joint2":
        dxl.setGoalPosition(forearm, joint_states_claw.position[i], UNIT_DEGREE);
        break;
      case "joint3":
        dxl.setGoalPosition(hand, joint_states_claw.position[i], UNIT_DEGREE);
        break;
      case "joint4":
        dxl.setGoalPosition(gripperOpener, joint_states_claw.position[i], UNIT_DEGREE);
        break;
      case "joint5":
        dxl.setGoalPosition(gripperTurner, joint_states_claw.position[i], UNIT_DEGREE);
        break;
    }
  }
  dxl.setGoalPosition(frontFlipper, position_frontFlipper, UNIT_DEGREE);
  dxl.setGoalPosition(backFlipper, position_backFlipper, UNIT_DEGREE);
}

