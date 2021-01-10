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
double refHPa = 1021.7;
char msgBuf[256];
double timeout;

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
  timeout = 0;
}

void loop() {
  if (SerialUSB.available()) {
    memset(msgBuf, 0, 256);
    int ix = 0;
    while (SerialUSB.available()) {
      char c = SerialUSB.read();
      delay(10);
      if (c > 31) msgBuf[ix++] = c;
    } msgBuf[ix] = 0;
    handleCommand();
  }
  if (millis() - timeout > 5000) {
    ClosedCube_BME680_Status status = readAndPrintStatus();
    if (status.newDataFlag) {
      Serial.print("result: ");
      double temp = bme680.readTemperature();
      double pres = bme680.readPressure();
      double hum = bme680.readHumidity();
      Serial.print("T=");
      Serial.print(temp);
      Serial.print("C, RH = ");
      Serial.print(hum);
      Serial.print("%, P = ");
      Serial.print(pres);
      Serial.print(" hPa");
      Serial.print(", Alt = ");
      // https://community.bosch-sensortec.com/t5/Question-and-answers/How-to-calculate-the-altitude-from-the-pressure-sensor-data/qaq-p/5702
      Serial.print(44330 * (1 - pow((pres / refHPa), (1 / 5.255))));
      Serial.print(" m");
      Serial.println();
      // Alternate calculation - same result
      // https://github.com/nodemcu/nodemcu-firmware/blob/release/app/modules/bme680.c#L535
      // Serial.print("Alt = ");
      // Serial.print((1.0 - pow(pres / refHPa, 1.0 / 5.25588)) / 2.25577e-5);
      // Serial.print(" m");
      // Serial.println();
      bme680.setForcedMode();
    }
    timeout = millis();
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
void handleCommand() {
  char c = msgBuf[0]; // Command
  if (c == '/') {
    c = msgBuf[1]; // Subcommand
    if (c == 'P') {
      double pp = (atof(msgBuf + 2));
      if (pp < 700) return;
      refHPa = pp;
      SerialUSB.print("set MSL pressure to "); SerialUSB.println(pp);
      return;
    }
  }
}
