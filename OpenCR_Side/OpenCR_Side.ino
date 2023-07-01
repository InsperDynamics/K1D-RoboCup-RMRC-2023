// K1D Robot (Redox 2)
// Insper Dynamics
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include "Sensors.h"
#include "Motors.h"
#include "Servos.h"
#include "IMU.h"

String current_command = "";
int current_value_1 = 0;
int current_value_2 = 0;
std_msgs::UInt16 gas;
std_msgs::Int8MultiArray temperature;



void commandCallback(const std_msgs::String& command){
  current_command = command.data;
}
void value1Callback(const std_msgs::Int16& value1){
  current_value_1 = value1.data;
}
void value2Callback(const std_msgs::Int16& value2){
  current_value_2 = value2.data;
}
void jointsCallback(const std_msgs::Int32MultiArray& joint_msg){
  ControlJointDynamixel(joint_msg.data);
}

ros::NodeHandle nodehandle;
ros::Publisher pub_temperature("temperature", &temperature);
ros::Publisher pub_gas("gas", &gas);
ros::Subscriber<std_msgs::String> sub_command("arduino_command", commandCallback);
ros::Subscriber<std_msgs::Int16> sub_value_1("arduino_value_1", value1Callback);
ros::Subscriber<std_msgs::Int16> sub_value_2("arduino_value_2", value2Callback);

void ControlMotors(String command, int command_parameter_1, int command_parameter_2) {
  if (command == "MotorsMove"){
    Move(command_parameter_1, command_parameter_2);
  }
  else if (command == "MotorsStop"){
    MotorsStop();
  }
  else if (command == "RaiseFrontFlippers"){
    RaiseFrontFlipper(command_parameter_1);
  }
  else if (command == "RaiseBackFlippers"){
    RaiseBackFlipper(command_parameter_1);
  }
  else if (command == "LowerFrontFlippers"){
    LowerFrontFlipper(command_parameter_1);
  }
  else if (command == "LowerBackFlippers"){
    LowerBackFlipper(command_parameter_1);
  }
  else if (command == "ClawOpen"){
    OpenGripper(command_parameter_1);
  }
  else if (command == "ClawClose"){
    CloseGripper(command_parameter_1);
  }
  else if (command == "First+"){
    FirstPlus(command_parameter_1);
  }
  else if (command == "First-"){
    FirstMinus(command_parameter_1);
  }
  else if (command == "Second+"){
    SecondPlus(command_parameter_1);
  }
  else if (command == "Second-"){
    SecondMinus(command_parameter_1);
  }
  else if (command == "Third+"){
    ThirdPlus(command_parameter_1);
  }
  else if (command == "Third-"){
    ThirdMinus(command_parameter_1);
  }
  else if (command == "SavePreset"){
    savePreset(command_parameter_1);
  }
  else if (command == "GotoPreset"){
    gotoPreset(command_parameter_1);
  }
  else if (command == "autoflip"){
    ControlFlipperDynamixel(flipper_auto);
  }
  else if (command == "defflip"){
    ControlFlipperDynamixel(flipper_position);
  }
}

void setup() {
  MotorsInitialize();
  SensorsInitialize();
  ServosInitialize();
  nodehandle.getHardware()->setBaud(115200);
  nodehandle.initNode();
  temperature.data_length = MLX90640_PIXEL_ARRAY_SIZE;
  temperature.data = (int8_t*)malloc(MLX90640_PIXEL_ARRAY_SIZE);
  nodehandle.advertise(pub_temperature);
  nodehandle.advertise(pub_gas);
  nodehandle.subscribe(sub_command);
  nodehandle.subscribe(sub_value_1);
  nodehandle.subscribe(sub_value_2);
}

void loop() {
  ReadSensors();
  temperature.data_length = MLX90640_PIXEL_ARRAY_SIZE;
  for (int i=0; i < MLX90640_PIXEL_ARRAY_SIZE; i++) {
    if isnan(MLX90640.frame[i]) {
      temperature.data[i] = 0;
    }
    else {
      temperature.data[i] = MLX90640.frame[i];
    }
  }
  gas.data = CO2level;
  pub_temperature.publish(&temperature);
  pub_gas.publish(&gas);
  nodehandle.spinOnce();
  delay(1);
  ControlMotors(current_command, current_value_1, current_value_2);
}
