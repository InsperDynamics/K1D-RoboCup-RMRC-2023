#include <DynamixelWorkbench.h>
#define DXL_BAUD              1000000

DynamixelWorkbench dxl;
uint8_t joint_id[4] = {11,12,13,14};
uint8_t gripper_id = 15;
uint8_t claw_id[5] = {11,12,13,14,15};
uint8_t flipper_id[4] = {1,2,3,4};
uint8_t *pid = &gripper_id;

int32_t flipper_position[4] = {};

int32_t joint_position[4] = {};
int32_t joint_velocity[4] = {};

int32_t gripper_position;

sensor_msgs::JointState joint_states;

void ServosInitialize()
{
  const char *log;
  Serial3.begin(DXL_BAUD);
  dxl.init("",DXL_BAUD);

  dxl.addSyncWriteHandler(claw_id[0], "Goal_Position", &log);
  dxl.addSyncWriteHandler(flipper_id[0], "Goal_Position", &log);
  dxl.addSyncReadHandler(128, 8);
  dxl.addSyncReadHandler(132, 4);

  for (int i = 0; i < 4; i++){
    uint8_t id = joint_id[i];
    uint8_t id_flipper = flipper_id[i];

    dxl.setVelocityBasedProfile(id);
    dxl.jointMode(id);
    dxl.setPositionControlMode(id_flipper);

    dxl.setVelocityBasedProfile(id_flipper);
    dxl.jointMode(id_flipper);
    dxl.setPositionControlMode(id_flipper);
  }
  dxl.setVelocityBasedProfile(gripper_id);
  dxl.currentBasedPositionMode(gripper_id, 200);
}

void ControlJointDynamixel(int32_t *joint_values)
{
  dxl.syncWrite(0 , joint_id, 4, joint_values, 1);
}

void ControlGripperDynamixel(int32_t gripper_value)
{  
  int32_t *p = &gripper_value;
  dxl.syncWrite(0, pid, 1, p, 1);
}

void ControlFlipperDynamixel(int32_t *flipper_values)
{
  dxl.syncWrite(1 , flipper_id, 4, flipper_values, 1);
}

void ReadJointValues()
{
  dxl.syncRead(0, joint_id, 4);
  dxl.getSyncReadData(0, joint_id, 4, 128, 4, joint_velocity);
  dxl.getSyncReadData(0, joint_id, 4, 132, 4, joint_position);
}

void GetJointState()
{
  ReadJointValues();
  for (int i = 0; i < 4; i++){
    float radian = dxl.convertValue2Radian(joint_id[i], joint_position[i]);
    joint_states.position[i] = radian;
    joint_states.velocity[i] = radian * 0.1047197551;
  }  
}

void ReadFlipperValues()
{
  dxl.syncRead(1, flipper_id, 4);
  dxl.getSyncReadData(1, flipper_id, 4, 132, 4, flipper_position);
}

void RaiseFrontFlipper(int delta)
{
  ReadFlipperValues();
  flipper_position[0] += delta;
  flipper_position[1] += delta;
  ControlFlipperDynamixel(flipper_position);  
}

void RaiseBackFlipper(int delta)
{
  ReadFlipperValues();
  flipper_position[2] += delta;
  flipper_position[3] += delta;
  ControlFlipperDynamixel(flipper_position);  
}

void LowerFrontFlipper(int delta)
{
  ReadFlipperValues();
  flipper_position[0] -= delta;
  flipper_position[1] -= delta;
  ControlFlipperDynamixel(flipper_position);  
}

void LowerBackFlipper(int delta)
{
  ReadFlipperValues();  
  flipper_position[2] -= delta;
  flipper_position[3] -= delta;
  ControlFlipperDynamixel(flipper_position);  
}
