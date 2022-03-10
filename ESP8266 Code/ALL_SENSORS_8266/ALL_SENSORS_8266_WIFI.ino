#include <DallasTemperature.h>

#include <MPU6050.h>
#include <heartRate.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <OneWire.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <Math.h>
#include "ThingSpeak.h"
#include "WiFiClient.h"
#include <ESP8266WiFi.h>

// Data wire is plugged TO GPIO 13
#define ONE_WIRE_BUS 13 //D7

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


const char* ssid = "Bhetri Sonia";   // your network SSID (name) 
const char* pass = "Sangpemenang2211";   // your network password
unsigned long myChannelNumber = 1111128;
const char* myWriteAPI = "5JFPEVS8J31EDKVA";
WiFiClient  client;

MAX30105 sensors2;
MPU6050 accelgyro;

// Number of temperature devices found
int numberOfDevices;

// We'll use this variable to store a found device address
DeviceAddress TempSensor_01 = {0x28, 0x78, 0xDE, 0x45, 0x3C, 0x19, 0x01, 0xDF};
DeviceAddress TempSensor_02 = {0x28, 0xF8, 0x79, 0x3B, 0x3C, 0x19, 0x01, 0xEA};
/*DeviceAddress TempSensor_01 = {0x28, 0x57, 0xD8, 0x5D, 0x3C, 0x19, 0x01, 0xF5};
DeviceAddress TempSensor_02 = {0x28, 0x8B, 0x6C, 0x46, 0x3C, 0x19, 0x01, 0xB0};*/

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte ledBrightness = 50; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication
const uint8_t scl = 5; //D1;
const uint8_t sda = 4; //D2;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ, ecg;
float temp1, temp2;
int error = 0;

void setup() {
  // start serial port
  Serial.begin(115200);

  // Start up the library
  MPU6050_Init();
  Wire.begin();
  sensors.begin();
  pinMode(14, INPUT); // Setup for leads off detection LO + ~D5
  pinMode(12, INPUT); // Setup for leads off detection LO - ~D6
  sensors.setResolution(TempSensor_01, 10);
  sensors.setResolution(TempSensor_02, 10);

  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("~~Serial Communication Connected Succesfully~~");
  Serial.println();
  Serial.println("~~Check All Available Sensors~~");
  Serial.println("Scanning...");

  if (sensors2.begin(Wire, I2C_SPEED_FAST)) {
    sensors2.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    Serial.println("Success to Initialize MAX30102");
  }
  else {
    Serial.println(F("MAX30102 was not found!!"));
    error++;
  }

  if (accelgyro.testConnection()) {
    Serial.println("Success to Initialize MPU6050!!");
  }
  else {
    Serial.println(F("MPU6050 was not found!!"));
    error++;
  }

  if (!(error == 0)) {
    Serial.println("PLEASE CHECK THE WIRE AND CONNECTION!!");
  }
  else {
    Serial.println("DONE!!...");
    delay(200);
  }
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);

  if (WiFi.status()!= WL_CONNECTED)
    {
     Serial.print("Attempting to connet to SSID:  ");
     Serial.println(ssid);
     while (WiFi.status() != WL_CONNECTED)
     {
      WiFi.begin(ssid,pass);
      Serial.print(".");
      delay(5000);
      }
      Serial.println("\nConnected.");
     }
}

void loop() {
  double Ax, Ay, Az, Gx, Gy, Gz, teta;

  //MAX30102
  bufferLength = 100;
  for (byte i = 25 ; i < bufferLength ; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }
  for (byte i = 75 ; i < bufferLength ; i++) {
    while (sensors2.available() == false)
      sensors2.check();

    redBuffer[i] = sensors2.getRed();
    irBuffer[i] = sensors2.getIR();
    sensors2.nextSample();



    //MPU6050
    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
    accelgyro.initialize();
    //divide each with their sensitivity scale factor
    Ax = (double)AccelX / AccelScaleFactor;
    Ay = (double)AccelY / AccelScaleFactor;
    Az = (double)AccelZ / AccelScaleFactor;
    //T =  (double)Temperature / 340 + 36.53; //temperature formula
    Gx = (double)GyroX / GyroScaleFactor;
    Gy = (double)GyroY / GyroScaleFactor;
    Gz = (double)GyroZ / GyroScaleFactor;
    teta = -(atan(Ax / Az)) * 180 / 3.14;

    //Sensor Suhu
    sensors.requestTemperatures(); // Send the command to get temperatures
    temp1 = sensors.getTempC(TempSensor_01);
    temp2 = sensors.getTempC(TempSensor_02);

        //AD8232
    if ((digitalRead(10) == 1) || (digitalRead(11) == 1)) { // 10 ~ 3v3, 11 ~ GND
      Serial.println('!');
    }
    else {
      // send the value of analog input 0:
      ecg=analogRead(A0);
      Serial.print("ECG: "); Serial.print(ecg);
    }

    Serial.print(" Teta: "); Serial.print(teta);
    Serial.print(" Temp S1: "); Serial.print(temp1);
    Serial.print(" Temp S2: "); Serial.print(temp2);

    Serial.print(", HR : "); Serial.print(heartRate, DEC);
    Serial.print(", SpO2 : "); Serial.println(spo2, DEC);

    ThingSpeak.setField(1,temp1);
    ThingSpeak.setField(2,heartRate);
    ThingSpeak.setField(4,spo2);
    ThingSpeak.setField(5,ecg);
    
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPI);
        if(x == 200)
        {
          Serial.println("Channel update successful.");
        }
        else{
          Serial.println("Problem updating channel. HTTP error code " + String(x));
        }
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init()
{
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
