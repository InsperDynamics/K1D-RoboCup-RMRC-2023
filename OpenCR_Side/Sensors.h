#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_CCS811.h>
Adafruit_AMG88xx ThermalImager;
Adafruit_CCS811 GasDetector;
int CO2level = 0;
float amg88_pixels[AMG88xx_PIXEL_ARRAY_SIZE];

void SensorsInitialize() {
	ThermalImager.begin();
	GasDetector.begin();
}

void ReadSensors() {
  GasDetector.readData();
	CO2level = GasDetector.geteCO2();
	ThermalImager.readPixels(amg88_pixels);
}
