#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial3

const int DXL_DIR_PIN = 84;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void FlippersInitialize(){

    dxl.begin(57600);

}

void Move(int id, int angle)
{
    dxl.setGoalPosition(id, angle, UNIT_DEGREE);
}

void ControlDynamixel(String command, int command_parameter_1, int command_parameter_2) {
  switch (command)
  {
  case "MoveFlipper":
    Move(command_parameter_1, command_parameter_2);
    break;
  default:
    break;
  }
}