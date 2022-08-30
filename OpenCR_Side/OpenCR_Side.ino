#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include "Sensors.h"
#include "Motors.h"
#include "Servos.h"

String current_command = "";
int current_value_1 = 0;
int current_value_2 = 0;
std_msgs::UInt16 gas;
std_msgs::Float32MultiArray temperature;
void commandCallback(const std_msgs::String& command){
  current_command = command.data;
}
void value1Callback(const std_msgs::UInt16& value1){
  current_value_1 = value1.data;
}
void value2Callback(const std_msgs::UInt16& value2){
  current_value_2 = value2.data;
}
ros::NodeHandle nodehandle;
ros::Publisher pub_temperature("temperature", &temperature);
ros::Publisher pub_gas("gas", &gas);
ros::Subscriber<std_msgs::String> sub_command("arduino_command", commandCallback);
ros::Subscriber<std_msgs::UInt16> sub_value_1("arduino_value_1", value1Callback);
ros::Subscriber<std_msgs::UInt16> sub_value_2("arduino_value_2", value2Callback);

void ControlMotors(String command, int command_parameter_1, int command_parameter_2) {
  if (command == "MoveBasearm"){
    MoveServo(0, command_parameter_1);
  } else if (command == "MoveForearm"){
    MoveServo(1, command_parameter_1);
  } else if (command == "MoveHand"){
    MoveServo(2, command_parameter_1);
  } else if (command == "MoveGripperTurner"){
    MoveServo(3, command_parameter_1);
  } else if (command == "MoveGripperOpener"){
    MoveServo(4, command_parameter_1);
  } else if (command == "RetractClaw"){
    RetractClaw();
  } else if (command == "ExtendClaw"){
    ExtendClaw();
  } else if (command == "MotorsStop"){
    MotorsStop();
  } else if (command == "MotorsRelease"){
    MotorsRelease();
  } else if (command == "MotorsMove"){
    Move(command_parameter_1, command_parameter_2);
  }
}

void setup() {
  MotorsInitialize();
  RetractClaw();
  SensorsInitialize();
  nodehandle.getHardware()->setBaud(115200);
  nodehandle.initNode();
  temperature.layout.dim[0].label = "temperature";
  temperature.layout.dim[0].size = AMG88xx_PIXEL_ARRAY_SIZE;
  temperature.layout.dim[0].stride = AMG88xx_PIXEL_ARRAY_SIZE;
  temperature.layout.data_offset = 0;
  temperature.data = (float *)malloc(sizeof(float)*AMG88xx_PIXEL_ARRAY_SIZE);
  nodehandle.advertise(pub_temperature);
  nodehandle.advertise(pub_gas);
  nodehandle.subscribe(sub_command);
  nodehandle.subscribe(sub_value_1);
  nodehandle.subscribe(sub_value_2);
}

void loop() {
  ReadSensors();
  temperature.data_length = AMG88xx_PIXEL_ARRAY_SIZE;
  for (int i=0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    temperature.data[i] = amg88_pixels[i];
  }
  gas.data = CO2level;
  pub_temperature.publish(&temperature);
  pub_gas.publish(&gas);
  nodehandle.spinOnce();
  delay(1);
  ControlMotors(current_command, current_value_1, current_value_2);
}
