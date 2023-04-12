#define enableA A0
#define enableB A1
#define enableC A2
#define enableD A3
#define motorUL 9
#define motorULdirA 10
#define motorULdirB 12
#define motorUR 11
#define motorURdirA 8
#define motorURdirB 13
#define motorLR 6
#define motorLRdirA 3
#define motorLRdirB 4
#define motorLL 5
#define motorLLdirA 7
#define motorLLdirB 2

void MotorsRelease(){
  digitalWrite(motorULdirA, LOW);
  digitalWrite(motorULdirB, LOW);
  digitalWrite(motorURdirA, LOW);
  digitalWrite(motorURdirB, LOW);
  digitalWrite(motorLLdirA, LOW);
  digitalWrite(motorLLdirB, LOW);
  digitalWrite(motorLRdirA, LOW);
  digitalWrite(motorLRdirB, LOW);
}
void MotorsStop(){
  digitalWrite(motorULdirA, HIGH);
  digitalWrite(motorULdirB, HIGH);
  digitalWrite(motorURdirA, HIGH);
  digitalWrite(motorURdirB, HIGH);
  digitalWrite(motorLLdirA, HIGH);
  digitalWrite(motorLLdirB, HIGH);
  digitalWrite(motorLRdirA, HIGH);
  digitalWrite(motorLRdirB, HIGH);
}

void MotorsInitialize(){
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(enableC, OUTPUT);
  pinMode(enableD, OUTPUT);
  digitalWrite(enableA, HIGH);
  digitalWrite(enableB, HIGH);
  digitalWrite(enableC, HIGH);
  digitalWrite(enableD, HIGH);
  pinMode(motorUL, OUTPUT);
  pinMode(motorULdirA, OUTPUT);
  pinMode(motorULdirB, OUTPUT);
  pinMode(motorUR, OUTPUT);
  pinMode(motorURdirA, OUTPUT);
  pinMode(motorURdirB, OUTPUT);
  pinMode(motorLL, OUTPUT);
  pinMode(motorLLdirA, OUTPUT);
  pinMode(motorLLdirB, OUTPUT);
  pinMode(motorLR, OUTPUT);
  pinMode(motorLRdirA, OUTPUT);
  pinMode(motorLRdirB, OUTPUT);
  MotorsRelease();
}

void URForward(){
  digitalWrite(motorURdirA, HIGH);
  digitalWrite(motorURdirB, LOW);
}
void URBackwards(){
  digitalWrite(motorURdirA, LOW);
  digitalWrite(motorURdirB, HIGH);
}
void ULForward(){
  digitalWrite(motorULdirA, HIGH);
  digitalWrite(motorULdirB, LOW);
}
void ULBackwards(){
  digitalWrite(motorULdirA, LOW);
  digitalWrite(motorULdirB, HIGH);
}
void LRForward(){
  digitalWrite(motorLRdirA, HIGH);
  digitalWrite(motorLRdirB, LOW);
}
void LRBackwards(){
  digitalWrite(motorLRdirA, LOW);
  digitalWrite(motorLRdirB, HIGH);
}
void LLForward(){
  digitalWrite(motorLLdirA, HIGH);
  digitalWrite(motorLLdirB, LOW);
}
void LLBackwards(){
  digitalWrite(motorLLdirA, LOW);
  digitalWrite(motorLLdirB, HIGH);
}

void Move(int speedL, int speedR){
  if(speedL > 0){
    ULForward();
    LLForward();
  }
  else {
    ULBackwards();
    LLBackwards();
  }
  if(speedR > 0){
    URForward();
    LRForward();
  }
  else {
    URBackwards();
    LRBackwards();
  }
  analogWrite(motorUL, constrain(abs(speedL), 0, 255));
  analogWrite(motorLL, constrain(abs(speedL), 0, 255));
  analogWrite(motorUR, constrain(abs(speedR), 0, 255));
  analogWrite(motorLR, constrain(abs(speedR), 0, 255));
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initialize");
  MotorsInitialize();
}

void loop() {
  Serial.println("Move");
  Move(120, 120);
  delay(1000);
  Serial.println("Stop");
  MotorsStop();
  delay(1000);
}
