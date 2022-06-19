#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include "src/Wire.h"
#include "src/Pangodream_18650_CL.h"
#include "SPIFFS.h" 
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <SPI.h>
#include "Button2.h"
#include "esp_adc_cal.h"
#include "bmp_NoiseCancelling_LOGO.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define rxPin 25
#define txPin 26
#define SWITCH1 32

#define RESTRICT_PITCH 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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

long baud = 9600;
char buff[512];
int vref = 1100;
int btnCick = false;

HardwareSerial HC12(2);
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 15

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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
        String sensorValue_roll = "Roll Value : " + String(ypr[2]);
        String sensorValue_pitch = "Pitch Value : " + String(ypr[1]);
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
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
//        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
 
void loop() {
  if (btnCick) {
//    showVoltage();
//    showSensorValue();
  }
    button_loop();
    
  if(digitalRead(SWITCH1) == HIGH) { 
//    mpu.enableSleep(true);
  }else{
//    mpu.enableSleep(false);
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
    String sensorValue_roll = "Roll Value : " + String(ypr[2]);
    String sensorValue_pitch = "Pitch Value : " + String(ypr[1]);
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
  Serial.print("3");
  Serial.print(",");
  HC12.print("3");
  HC12.print(",");
}

void serialPrintDataKalmanFilter(void) {if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print(",");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(",");
            Serial.println(ypr[0] * 180/M_PI);

        #endif
    }
    delay(10);
}

void hc12PrintDataKalmanFilter(void) {// read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            HC12.print(ypr[2] * 180/M_PI);
            HC12.print(",");
            HC12.print(ypr[1] * 180/M_PI);
            HC12.print(",");
            HC12.println(ypr[0] * 180/M_PI);

        #endif
    }
    delay(10);
}
