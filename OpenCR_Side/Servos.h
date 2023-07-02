#include <DynamixelWorkbench.h>
#include <map>
#define DXL_BAUD 1000000

DynamixelWorkbench dxl;
uint8_t joint_id[3] = {12,13,14};
uint8_t gripper_id = 15;
uint8_t claw_id[4] = {12,13,14,15};
uint8_t flipper_id[4] = {1,2,3,4};
uint8_t *pid = &gripper_id;

int32_t flipper_position[4] = {2048,2048,2048,2048};
int32_t flipper_position_default[4] = {2048,2048,2048,2048};
int32_t flipper_auto[4] = {1500,2500,1500,2500};

int32_t joint_position[3] = {2600, 1125, 1850};
int32_t joint_position_default[3] = {2600, 1125, 1850};
int32_t joint_velocity[3] = {};
int32_t gripper_position = 0;

std::map<int, int32_t*> preset;

void ControlJointDynamixel(int32_t *joint_pos)
{
  int32_t joint_values[3];
  for (int i = 0; i < 3; i++) {
    dxl.goalPosition(joint_id[i], joint_pos[i]);
  }
}

void ControlGripperDynamixel(int32_t gripper_value)
{
  dxl.goalPosition(gripper_id, (int32_t) gripper_position);
}

void ControlFlipperDynamixel(int32_t *flipper_values)
{
  for (size_t i = 0; i < 4; i++)
  {
    dxl.goalPosition(flipper_id[i], flipper_values[i]);
  }
}

void RaiseFrontFlipper(int delta)
{
  flipper_position[0] += delta;
  flipper_position[1] -= delta;
  ControlFlipperDynamixel(flipper_position);  
}

void RaiseBackFlipper(int delta)
{
  flipper_position[2] += delta;
  flipper_position[3] -= delta;
  ControlFlipperDynamixel(flipper_position);  
}

void LowerFrontFlipper(int delta)
{
  flipper_position[0] -= delta;
  flipper_position[1] += delta;
  ControlFlipperDynamixel(flipper_position);  
}

void LowerBackFlipper(int delta)
{
  flipper_position[2] -= delta;
  flipper_position[3] += delta;
  ControlFlipperDynamixel(flipper_position);  
}

void LowerFlipper(int idx, int delta) 
{
  if (idx == 0 || idx == 2) {
    flipper_position[idx] -= delta;
  } else {
    flipper_position[idx] += delta;
  }
  ControlFlipperDynamixel(flipper_position); 
}

void RaiseFlipper(int idx, int delta) 
{
  if (idx == 0 || idx == 2) {
    flipper_position[idx] += delta;
  } else {
    flipper_position[idx] -= delta;
  }
  ControlFlipperDynamixel(flipper_position); 
}

void OpenGripper(int delta)
{
  if (gripper_position >= 0) {
    gripper_position -= delta;
    ControlGripperDynamixel(gripper_position);    
  }  
}

void CloseGripper(int delta)
{
  if (gripper_position < 2048) {
    gripper_position += delta;
    ControlGripperDynamixel(gripper_position);
  }  
}

void FirstPlus(int delta)
{ 
  if (joint_position[0] < 2600) {
    joint_position[0] += delta;
    ControlJointDynamixel(joint_position);
  }  
}

void FirstMinus(int delta)
{
  joint_position[0] -= delta;
  ControlJointDynamixel(joint_position);  
}

void SecondPlus(int delta)
{
  joint_position[1] += delta;
  ControlJointDynamixel(joint_position); 
}

void SecondMinus(int delta)
{
  joint_position[1] -= delta;
  ControlJointDynamixel(joint_position);  
}

void ThirdPlus(int delta)
{
  joint_position[2] += delta;
  ControlJointDynamixel(joint_position);  
}

void ThirdMinus(int delta)
{
  joint_position[2] -= delta;
  ControlJointDynamixel(joint_position); 
}

void ClawRetract()
{
  ControlFlipperDynamixel(flipper_position_default);
  ControlJointDynamixel(joint_position_default);
}

void ServosInitialize()
{
  const char *log;
  dxl.init("",DXL_BAUD);
  for (int i = 0; i < 4; i++){

    if (i < 3){
      uint8_t id = joint_id[i];
      dxl.ping(id);
      dxl.jointMode(id);
      dxl.setPositionControlMode(id);
    }

    uint8_t id_flipper = flipper_id[i];

    dxl.ping(id_flipper);
    dxl.jointMode(id_flipper);
  }
  dxl.ping(gripper_id);
  dxl.jointMode(gripper_id);
  dxl.setPositionControlMode(gripper_id);
  ControlFlipperDynamixel(flipper_position);
  ControlGripperDynamixel(gripper_position);
  ControlJointDynamixel(joint_position);
}
