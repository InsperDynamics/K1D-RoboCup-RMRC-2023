#include <Dynamixel2Arduino.h>
#define DXL_DIR_PIN 84
#define basearm_A 0
#define basearm_B 1
#define forearm 2
#define hand 3
#define gripperOpener 4
#define gripperTurner 5 

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

void MoveServo(int servo, int pos){
  switch(servo){
    case 0:
      dxl.setGoalPosition(basearm_A, constrain(pos, 60, 120), UNIT_DEGREE);
      dxl.setGoalPosition(basearm_B, constrain(180 - pos, 60, 120), UNIT_DEGREE);
      break;
    case 1:
      dxl.setGoalPosition(forearm, constrain(pos, 20, 120), UNIT_DEGREE);
      break;
    case 2:
      dxl.setGoalPosition(hand, constrain(pos, 0, 180), UNIT_DEGREE);
      break;
    case 3:
      dxl.setGoalPosition(gripperOpener, constrain(pos, 0, 180), UNIT_DEGREE);
      break;
    case 4:
      dxl.setGoalPosition(gripperTurner, constrain(pos, 20, 60), UNIT_DEGREE);
      break;
  }
}

