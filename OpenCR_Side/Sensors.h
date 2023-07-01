#include <Wire.h>
#include <RAK12052-MLX90640.h>
#include <DFRobot_CCS811.h>
#define MLX90640_PIXEL_ARRAY_SIZE 768
RAK_MLX90640 MLX90640;
DFRobot_CCS811 GasDetector;
int CO2level = 0;

void SensorsInitialize() {
  MLX90640.begin();
  Wire.begin();
  Wire.setClock(500000);
  MLX90640.setMode(MLX90640_CHESS);
  MLX90640.setResolution(MLX90640_ADC_18BIT);
  MLX90640.setRefreshRate(MLX90640_16_HZ);
	GasDetector.begin();
}

void ReadSensors() {
  if (GasDetector.checkDataReady() == true){
    CO2level = GasDetector.getCO2PPM();
  }
	MLX90640.getFrame(MLX90640.frame);
}
