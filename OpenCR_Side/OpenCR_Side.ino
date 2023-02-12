// K1D Robot (Redox 2)
// Insper Dynamics
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "Sensors.h"
#include "Motors.h"
#include "Flippers.h"
#include "IMU.h"
#include "Encoders.h"
#include "usb_to_dxl.h"

String current_command = "";
int current_value_1 = 0;
int current_value_2 = 0;
sensor_msgs::JointState joint_states_claw;
float position_frontFlipper;
float position_backFlipper;
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
  switch (command)
  {
  case "MotorsMove":
    Move(command_parameter_1, command_parameter_2);
    break;
  case "MotorsStop":
    MotorsStop();
    break;
  default:
    break;
  }
}

void setup() {
  MotorsInitialize();
  RetractClaw();
  SensorsInitialize();
  CalibrateIMU();
  SerialInitialize();
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
  DxlSerial();
  UpdateIMU();
  UpdateEncoders();
  temperature.data_length = 32*24;
  for (int i=0; i < 32*24; i++) {
    temperature.data[i] = mlx90640_pixels[i];
  }
  gas.data = CO2level;
  pub_temperature.publish(&temperature);
  pub_gas.publish(&gas);
  nodehandle.spinOnce();
  delay(1);
  ControlMotors(current_command, current_value_1, current_value_2);
  ControlDynamixel(current_command, current_value_1, current_value_2);
}
