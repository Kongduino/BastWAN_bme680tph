/**************************************************************************************
  This is an example for ClosedCube BME680 Humidity, Temperature, Pressure and
  Gas Sensor Breakout Board
  Initial Date: 15-Sep-2017
  Hardware connections for Arduino Uno:
  VDD to 3.3V DC
  SCL to A5
  SDA to A4
  GND to common ground
  Written by AA for ClosedCube
  MIT License
**************************************************************************************/
#include <Wire.h>
#include "ClosedCube_BME680.h"

ClosedCube_BME680 bme680;

void setup() {
  Wire.begin();
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  SerialUSB.begin(115200);
  SerialUSB.flush();
  delay(3000);
  Serial.println("\nClosedCube BME680 ([T]emperature, [H]umidity, [P]ressure) Arduino Test");
  bme680.init(0x77); // I2C address: 0x76 or 0x77
  bme680.reset();
  Serial.print("Chip ID=0x");
  Serial.println(bme680.getChipID(), HEX);
  // oversampling: humidity = x1, temperature = x2, pressure = x16
  bme680.setOversampling(BME680_OVERSAMPLING_X1, BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X16);
  bme680.setIIRFilter(BME680_FILTER_3);
  bme680.setForcedMode();
}

void loop() {
  ClosedCube_BME680_Status status = readAndPrintStatus();
  if (status.newDataFlag) {
    Serial.print("result: ");
    double temp = bme680.readTemperature();
    double pres = bme680.readPressure();
    double hum = bme680.readHumidity();
    Serial.print("T=");
    Serial.print(temp);
    Serial.print("C, RH=");
    Serial.print(hum);
    Serial.print("%, P=");
    Serial.print(pres);
    Serial.print(" hPa");
    Serial.println();
    delay(5000); // let's do nothing and wait a bit before perform next measurements
    bme680.setForcedMode();
  } else {
    delay(1000); // sensor data not yet ready
  }
}

ClosedCube_BME680_Status readAndPrintStatus() {
  ClosedCube_BME680_Status status = bme680.readStatus();
  //  Serial.print("status: (");
  //  Serial.print(status.newDataFlag);
  //  Serial.print(",");
  //  Serial.print(status.measuringStatusFlag);
  //  Serial.print(",");
  //  Serial.print(status.gasMeasuringStatusFlag);
  //  Serial.print(",");
  //  Serial.print(status.gasMeasurementIndex);
  //  Serial.println(") (newDataFlag,StatusFlag,GasFlag,GasIndex)");
  return status;
}
