#include "src/HardwareSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define rxPin 25
#define txPin 26
#define SWITCH1 32

long baud = 9600;

HardwareSerial HC12(2);
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;
void printAvailableData();
void serialPrintAvailableData();
 
void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  Serial.begin(baud); 
  HC12.begin(baud, SERIAL_8N1, rxPin, txPin);             // 시리얼 모니터 속도 정의
  
  // MPU6050 센서 I2C 통신 테스트
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}
 
void loop() {
  if(digitalRead(SWITCH1) == HIGH) { 
    mpu.enableSleep(true);
    Serial.println("SWITCH OFF"); 
  }else{
    mpu.enableSleep(false);
    sender();
//    printAvailableData();
    serialPrintAvailableData();
    delay(1000);
  }
  
//  while (HC12.available()) {        // HC-12에 수신 데이터가 존재하는 경우
//    Serial.write(HC12.read());      // HC-12 모듈의 출력 내용을 읽어 시리얼 모니터로 전송
//  }
//  while (Serial.available()) {      // 시리얼 모니터의 입력 내용이 존재하면
//    HC12.write(Serial.read());      // 읽어서 HC-12 모듈로 전송
//  }
}

void sender(void) {
  Serial.print("Sender01 : ");
}

void printAvailableData(void) {
  mpu.getEvent(&a, &g, &temp);

  acceleration_x = a.acceleration.x;
  acceleration_y = a.acceleration.y;
  acceleration_z = a.acceleration.z;
  gyro_x = g.gyro.x;
  gyro_y = g.gyro.y;
  gyro_z = g.gyro.z;
  HC12.print("Acceleration X: ");
  HC12.print(acceleration_x);
  HC12.print(", Y: ");
  HC12.print(acceleration_y);
  HC12.print(", Z: ");
  HC12.print(acceleration_z);
  HC12.println(" m/s^2");

  HC12.print("Rotation X: ");
  HC12.print(gyro_x);
  HC12.print(", Y: ");
  HC12.print(gyro_y);
  HC12.print(", Z: ");
  HC12.print(gyro_z);
  HC12.println(" rad/s");

  HC12.print("");
}

void serialPrintAvailableData(void) {
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acc: ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rot: ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("");
}
