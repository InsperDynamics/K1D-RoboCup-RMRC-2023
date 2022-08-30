#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_CCS811.h>
Adafruit_MLX90640 ThermalImager;
Adafruit_CCS811 GasDetector;
int CO2level = 0;
float mlx90640_pixels[32*24];

void SensorsInitialize() {
	ThermalImager.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
  ThermalImager.setResolution(MLX90640_ADC_18BIT);
  ThermalImager.setRefreshRate(MLX90640_8_HZ);
	GasDetector.begin();
}

void ReadSensors() {
  GasDetector.readData();
	CO2level = GasDetector.geteCO2();
	ThermalImager.getFrame(mlx90640_pixels);
}
