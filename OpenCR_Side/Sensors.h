#include <Wire.h>
#include <Melopero_AMG8833.h>
#include <DFRobot_CCS811.h>
#define AMG88xx_PIXEL_ARRAY_SIZE 64
Melopero_AMG8833 ThermalImager;
DFRobot_CCS811 GasDetector;
float amg8833_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int CO2level = 0;

void SensorsInitialize() {
  GasDetector.begin();
  Wire.begin();
  ThermalImager.initI2C();
  ThermalImager.resetFlagsAndSettings();
  ThermalImager.setFPSMode(FPS_MODE::FPS_10);
}

void ReadSensors() {
  if (GasDetector.checkDataReady() == true){
    CO2level = GasDetector.getCO2PPM();
  }
  ThermalImager.updatePixelMatrix();
  for (int x = 0; x < 8; x++){
    for (int y = 0; y < 8; y++){
      amg8833_pixels[(8*x) + y] = ThermalImager.pixelMatrix[y][x];
    }
  }
}
