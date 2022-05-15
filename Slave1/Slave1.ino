#include "src/HardwareSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "Button2.h"
#include "esp_adc_cal.h"
#include "bmp.h"

#define rxPin 25
#define txPin 26
#define SWITCH1 32

// 디스플레이를 통해 전압확인과 동작을 위한 핀 선언
#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

long baud = 9600;
char buff[512];
int vref = 1100;
int btnCick = false;


HardwareSerial HC12(2);
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;
void printAvailableData();
void serialPrintAvailableData();

void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage,  tft.width() / 2, tft.height() / 2 );
    }
}

void button_init()
{
    btn1.setPressedHandler([](Button2 & b) {
        Serial.println("Detect Voltage..");
        btnCick = true;
    });
    btn2.setPressedHandler([](Button2 & b) {
        btnCick = false;
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        espDelay(6000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
        delay(200);
        esp_deep_sleep_start();
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}

 
void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  Serial.begin(baud); 
  HC12.begin(baud, SERIAL_8N1, rxPin, txPin);             // 시리얼 모니터 속도 정의

  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  tft.setSwapBytes(true);
  tft.pushImage(0, 0,  240, 135, ttgo);
  espDelay(5000);

  button_init();
  
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
  if (btnCick) {
        showVoltage();
    }
    button_loop();
  
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
