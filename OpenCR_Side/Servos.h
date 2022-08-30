#include <Servo.h>
#define basearmA_pin 6
#define basearmB_pin 7
#define forearm_pin 4
#define hand_pin 5
#define gripperTurner_pin 3
#define gripperOpener_pin 2
Servo basearmA, basearmB, forearm, hand, gripperTurner, gripperOpener;
bool extended = false;

void MoveServo(int servo, int pos){
  switch(servo){
    case 0:
      basearmA.write(constrain(pos, 60, 120));
      basearmB.write(constrain(180 - pos, 60, 120));
      break;
    case 1:
      forearm.write(constrain(pos, 20, 120));
      break;
    case 2:
      hand.write(constrain(pos, 0, 180));
      break;
    case 3:
      gripperTurner.write(constrain(pos, 0, 180));
      break;
    case 4:
      gripperOpener.write(constrain(pos, 20, 60));
      break;
  }
}

void AttachServos(){
    basearmA.attach(basearmA_pin);
    basearmB.attach(basearmB_pin);
    forearm.attach(forearm_pin);
    hand.attach(hand_pin);
    gripperOpener.attach(gripperOpener_pin);
    gripperTurner.attach(gripperTurner_pin);
}

void DetachServos(){
    basearmA.detach();
    basearmB.detach();
    forearm.detach();
    hand.detach();
    gripperOpener.detach();
    gripperTurner.detach();
}

void RetractClaw(){
    forearm.write(0);
    hand.write(0);
    basearmA.write(130);
    basearmB.write(50);
    gripperOpener.write(60);
    gripperTurner.write(90);
    extended = false;
    delay(3000);
    DetachServos();
}

void ExtendClaw(){
    AttachServos();
    forearm.write(50);
    hand.write(180);
    basearmA.write(70);
    basearmB.write(110);
    gripperOpener.write(20);
    gripperTurner.write(90);
    extended = true;
}
