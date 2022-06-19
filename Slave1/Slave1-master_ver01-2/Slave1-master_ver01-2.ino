#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "src/Wire.h"
#include "Kalman.h"
#include "src/Pangodream_18650_CL.h"
#include "SPIFFS.h" 
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <SPI.h>
#include "Button2.h"
#include "esp_adc_cal.h"
//#include "bmp.h"
#include "bmp_NoiseCancelling_LOGO.h"

#define rxPin 25
#define txPin 26
#define SWITCH1 32

#define RESTRICT_PITCH 

Kalman kalmanX; // 칼만 객체 생성. 
Kalman kalmanY;

// 디스플레이를 통해 전압확인과 동작을 위한 핀 선언
#define ADC_EN              14 
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

// 배터리 아이콘 크기의 정의
#define ICON_WIDTH 70
#define ICON_HEIGHT 36
#define STATUS_HEIGHT_BAR ICON_HEIGHT
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define ICON_POS_X (tft.width() - ICON_WIDTH)

// ADC 전압 값
#define MIN_USB_VOL 4.9   // 최소 usb전압 값, USB 연결 확인을 위함.
#define ADC_PIN 34  
#define CONV_FACTOR 1.8   // 계수
#define READS 20          // 정확한 판독을 위해 20회 반복

Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);
char *batteryImages[] = {"/battery_01.jpg", "/battery_02.jpg", "/battery_03.jpg", "/battery_04.jpg", "/battery_05.jpg"};

TFT_eSPI tft = TFT_eSPI(135, 240);
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

//#define BLACK    0x0000
//#define BLUE     0x001F
//#define RED      0xF800
//#define GREEN    0x07E0
//#define CYAN     0x07FF
//#define MAGENTA  0xF81F
//#define YELLOW   0xFFE0 
//#define WHITE    0xFFFF

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;
double compAngleX, compAngleY; // 상보필터 적용 변수 선언
double kalAngleX, kalAngleY; // 칼만필터 적용 변수 선언

double roll;
double pitch;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

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
//        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage,  tft.width() / 2, tft.height() / 3 );
    }
}

void showSensorValue()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
//        Serial.println(String(roll));
        String sensorValue_roll = "Roll Value : " + String(roll);
        String sensorValue_pitch = "Pitch Value : " + String(pitch);
//        tft.drawString(sensorValue_roll,  tft.width() / 2, tft.height() / 2 );
//        tft.drawString(sensorValue_pitch,  tft.width() / 2, tft.height() / 1.5 );
        tft.println(sensorValue_roll);
        tft.println(sensorValue_pitch);
    }
}

void button_init()
{
    btn1.setPressedHandler([](Button2 & b) {
        btnCick = true;
        tft.fillScreen(TFT_BLACK);
        xTaskCreate(battery_info, "battery_info", 2048, NULL, 1, NULL);
    });
    btn2.setPressedHandler([](Button2 & b) {
        btnCick = false;
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE,TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        espDelay(6000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
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
  
//  pinMode(ADC_EN, OUTPUT);
//  digitalWrite(ADC_EN, HIGH);

  pinoutInit();
  SPIFFSInit();
  displayInit();
  button_init();
  
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
//    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}
 
void loop() {
  if (btnCick) {
//    showVoltage();
//    showSensorValue();
  }
    button_loop();
    
  if(digitalRead(SWITCH1) == HIGH) { 
    mpu.enableSleep(true);
  }else{
    mpu.enableSleep(false);
    sender();
    hc12PrintDataKalmanFilter();
    serialPrintDataKalmanFilter();
  }
}

void pinoutInit(){    // 전압을 측정하기 위해 켜놓는 것. 소비전력이 많으면 LOW.
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
}


void SPIFFSInit(){
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
    while (1) yield(); // Stay here twiddling thumbs waiting
  }
  Serial.println("\r\nInitialisation done.");
}

void displayInit(){   // 디스플레이 초기화
  tft.begin();
  tft.init();
  tft.setRotation(1);
  tft.setTextColor(TFT_WHITE,TFT_BLACK); 
//  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.setTextFont(2.5);
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);

  tft.setSwapBytes(true);
  tft.pushImage(0, 0,  240, 135, ttgo);
  espDelay(1000);
}

void battery_info(void *arg)
{
  while (1) {
    tft.setCursor (0, STATUS_HEIGHT_BAR);
    tft.println("");
//    tft.print("Average value from pin: ");
//    tft.println(BL.pinRead());
    tft.print("Volts: ");
    tft.println(BL.getBatteryVolts());
    tft.print("Charge level: ");
    tft.println(BL.getBatteryChargeLevel());
    String sensorValue_roll = "Roll Value : " + String(roll);
    String sensorValue_pitch = "Pitch Value : " + String(pitch);
    tft.println(sensorValue_roll);
    tft.println(sensorValue_pitch);
    
    if(BL.getBatteryVolts() >= MIN_USB_VOL){
      for(int i=0; i< ARRAY_SIZE(batteryImages); i++){
        drawingBatteryIcon(batteryImages[i]);
        drawingText("Chrg");
        vTaskDelay(500);
      }
    }else{
        int imgNum = 0;
        int batteryLevel = BL.getBatteryChargeLevel();
        if(batteryLevel >=80){
          imgNum = 3;
        }else if(batteryLevel < 80 && batteryLevel >= 50 ){
          imgNum = 2;
        }else if(batteryLevel < 50 && batteryLevel >= 20 ){
          imgNum = 1;
        }else if(batteryLevel < 20 ){
          imgNum = 0;
        }  
    
        drawingBatteryIcon(batteryImages[imgNum]);    
        drawingText(String(batteryLevel) + "%");
        vTaskDelay(500);
    }
//      tft.print("Never Used Stack Size: ");
//      tft.println(uxTaskGetStackHighWaterMark(NULL));
    }  
}

void drawingBatteryIcon(String filePath){
   TJpgDec.drawFsJpg(ICON_POS_X, 0, filePath);
}

void drawingText(String text){
  tft.fillRect(0, 0, ICON_POS_X, ICON_HEIGHT,TFT_BLACK);
  tft.setTextDatum(5);
  tft.drawString(text, ICON_POS_X-2, STATUS_HEIGHT_BAR/2, 4);
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if ( y >= tft.height() ) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

void sender(void) {
  Serial.print("2");
  Serial.print(",");
  HC12.print("2");
  HC12.print(",");
}

void serialPrintDataKalmanFilter(void) {
/* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  Serial.print(roll); 
  Serial.print(",");
  Serial.print(pitch); 
  Serial.print(",");

  Serial.print("\r\n");
  delay(10);
}

void hc12PrintDataKalmanFilter(void) {
/* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  HC12.print(roll); 
  HC12.print(",");

  HC12.println(pitch);

//  HC12.print("\r\n");
  delay(20);
}
