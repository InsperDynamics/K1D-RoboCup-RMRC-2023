#include <DynamixelWorkbench.h>
#define DXL_BAUD              1000000

DynamixelWorkbench dxl;
uint8_t dxl_joint[4] = {11,12,13,14};
uint8_t id_array[5] = {11,12,13,14,15};
uint8_t gripper_id = 15;
uint8_t *pid = &gripper_id;
int32_t position_values[5] = {};
int32_t velocity_values[5] = {};
int32_t current_values[5] = {}; 


void ServosInitialize()
{
  const char *log;
  Serial3.begin(DXL_BAUD);
  dxl.init("",DXL_BAUD);
  dxl.addSyncWriteHandler(dxl_joint[0], "Goal_Position", &log);
  dxl.addSyncReadHandler(126, 10); 
  for (int i = 0; i < 4; i++){
    uint8_t id = dxl_joint[i];
    dxl.setVelocityBasedProfile(id);
    dxl.jointMode(id);
  }
  dxl.setVelocityBasedProfile(gripper_id);
  dxl.currentBasedPositionMode(gripper_id, 200);
}

void ControlJointDynamixel(long int *joint_values)
{
  dxl.syncWrite(0 , dxl_joint, 4, joint_values, 1);
}

void ControlGripperDynamixel(long int gripper_value)
{  
  int32_t *p = &gripper_value;
  dxl.syncWrite(0, pid, 1, p, 1);
}

void ReadJointValues()
{
  dxl.syncRead(0, id_array, 5);
  dxl.getSyncReadData(0, id_array, 5, 126, 2, current_values);
  dxl.getSyncReadData(0, id_array, 5, 128, 4, velocity_values);
  dxl.getSyncReadData(0, id_array, 5, 132, 4, position_values);
}

