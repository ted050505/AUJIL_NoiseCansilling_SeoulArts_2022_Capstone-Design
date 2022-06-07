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
#include "I2Cdev.h"
#include "MPU6050.h"

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

unsigned long now, lastTime = 0;
float dt;                                   // Differential time

int16_t ax, ay, az, gx, gy, gz;             // Accelerometer gyroscope raw data
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    // Angle variable
int roll=0, pitch=0, yaw=0;
long axo = 0, ayo = 0, azo = 0;             // Accelerometer offset
long gxo = 0, gyo = 0, gzo = 0;             // Gyro offset

float pi = 3.1415926;
float AcceRatio = 16384.0;                  // Accelerometer scale factor
float GyroRatio = 131.0;                    // Gyroscope scale factor

uint8_t n_sample = 8;                       // Accelerometer filter algorithm sampling number
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};  // x,y-axis sampling queue
long aax_sum, aay_sum,aaz_sum;                      // x,y-axis sampling add

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; // Accelerometer covariance calculation queue
float Px=1, Rx, Kx, Sx, Vx, Qx;             // x-axis Calman variables
float Py=1, Ry, Ky, Sy, Vy, Qy;             // y-axis Calman variables
float Pz=1, Rz, Kz, Sz, Vz, Qz;             // z-axis Calman variables

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

long baud = 9600;
char buff[512];
int vref = 1100;
int btnCick = false;

HardwareSerial HC12(2);
MPU6050 accelgyro;

sensors_event_t a, g, temp;

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

  accelgyro.initialize();                 // initialization

  unsigned short times = 200;             // The number of samples
  for(int i=0;i<times;i++)
  {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read the six-axis original value
        axo += ax; ayo += ay; azo += az;      // Sampling
        gxo += gx; gyo += gy; gzo += gz;
  }
    
    axo /= times; ayo /= times; azo /= times; // Calculate accelerometer offset
    gxo /= times; gyo /= times; gzo /= times; // Calculate the gyro offset
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
    serialPrintDataKalman_RollPitchYaw();
    hc12DataKalman_RollPitchYaw();
    delay(10);
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
  Serial.print("ch01");
  Serial.print(",");
  HC12.print("ch01");
  HC12.print(",");
}

void serialPrintDataKalman_RollPitchYaw() {
  unsigned long now = millis();             // current time(ms)
    dt = (now - lastTime) / 1000.0;           // Differential time(s)
    lastTime = now;                           // Last sampling time(ms)

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read the six-axis original value

    float accx = ax / AcceRatio;              // x-axis acceleration
    float accy = ay / AcceRatio;              // y-axis acceleration
    float accz = az / AcceRatio;              // z-axis acceleration

    aax = atan(accy / accz) * (-180) / pi;    // The x-axis angle to the z-axis
    aay = atan(accx / accz) * 180 / pi;       // The y-axis angle to the z-axis
    aaz = atan(accz / accy) * 180 / pi;       // The z-axis angle to the y-axis

    aax_sum = 0;                              // Sliding weight filtering algorithm for accelerometer raw data
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; // Angle AM ​​to 0-90 °
    aays[n_sample-1] = aay;                        // Here we use the experimental method to obtain the appropriate coefficient
    aay_sum += aay * n_sample;                     // This example factor is 9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt; // x-axis angular velocity
    float gyroy = - (gy-gyo) / GyroRatio * dt; // x-axis angular velocity
    float gyroz = - (gz-gzo) / GyroRatio * dt; // x-axis angular velocity
    agx += gyrox;                             // x-axis angular velocity integral
    agy += gyroy;                             // y-axis angular velocity integral
    agz += gyroz;                             // z-axis angular velocity integral
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //The average value of the calculation
        a_x[i-1] = a_x[i];                      // The acceleration average
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 // x-axis acceleration average
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 // y-axis acceleration average
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;                                 // z-axis acceleration average

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              // Get the variance
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         // 0.0025 in the following instructions ...
    Kx = Px / (Px + Rx);                      // Calculate the Kalman gain
    agx = agx + Kx * (aax - agx);             // Gyro angle and accelerometer speed superimposed
    Px = (1 - Kx) * Px;                       // Update p value

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

    /* kalman end */
   roll = round(agx);
   pitch = round(agy);
   yaw = round(agz);

   Serial.print(roll);Serial.print(",");
   Serial.print(pitch);Serial.print(",");
   Serial.print(yaw);Serial.println();
}

void hc12DataKalman_RollPitchYaw() {
  unsigned long now = millis();             // current time(ms)
    dt = (now - lastTime) / 1000.0;           // Differential time(s)
    lastTime = now;                           // Last sampling time(ms)

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read the six-axis original value

    float accx = ax / AcceRatio;              // x-axis acceleration
    float accy = ay / AcceRatio;              // y-axis acceleration
    float accz = az / AcceRatio;              // z-axis acceleration

    aax = atan(accy / accz) * (-180) / pi;    // The x-axis angle to the z-axis
    aay = atan(accx / accz) * 180 / pi;       // The y-axis angle to the z-axis
    aaz = atan(accz / accy) * 180 / pi;       // The z-axis angle to the y-axis

    aax_sum = 0;                              // Sliding weight filtering algorithm for accelerometer raw data
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; // Angle AM ​​to 0-90 °
    aays[n_sample-1] = aay;                        // Here we use the experimental method to obtain the appropriate coefficient
    aay_sum += aay * n_sample;                     // This example factor is 9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt; // x-axis angular velocity
    float gyroy = - (gy-gyo) / GyroRatio * dt; // x-axis angular velocity
    float gyroz = - (gz-gzo) / GyroRatio * dt; // x-axis angular velocity
    agx += gyrox;                             // x-axis angular velocity integral
    agy += gyroy;                             // y-axis angular velocity integral
    agz += gyroz;                             // z-axis angular velocity integral
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //The average value of the calculation
        a_x[i-1] = a_x[i];                      // The acceleration average
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 // x-axis acceleration average
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 // y-axis acceleration average
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;                                 // z-axis acceleration average

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              // Get the variance
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         // 0.0025 in the following instructions ...
    Kx = Px / (Px + Rx);                      // Calculate the Kalman gain
    agx = agx + Kx * (aax - agx);             // Gyro angle and accelerometer speed superimposed
    Px = (1 - Kx) * Px;                       // Update p value

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

    /* kalman end */
   roll = round(agx);
   pitch = round(agy);
   yaw = round(agz);

   HC12.print(roll);HC12.print(",");
   HC12.print(pitch);HC12.print(",");
   HC12.print(yaw);HC12.println();
}
