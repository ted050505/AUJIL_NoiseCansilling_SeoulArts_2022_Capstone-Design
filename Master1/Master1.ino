//#include <HardwareSerial.h>
//#include <SoftwareSerial.h>

//HardwareSerial HC12(2);
//SoftwareSerial HC12(2, 3);

//HardwareSerial Pin
#define rxPin 0

//SoftwareSerial Pin
//#define rxPin 2
//#define txPin 3

void setup() {
  pinMode(rxPin, INPUT);
//      pinMode(txPin, OUTPUT);
  Serial.begin(9600);             // 시리얼 모니터 속도 정의
}

void loop() {
  while (Serial.available()) {        // HC-12에 수신 데이터가 존재하는 경우
    Serial.write(Serial.read());      // HC-12 모듈의 출력 내용을 읽어 시리얼 모니터로 전송
  }
//    while (Serial.available()) {      // 시리얼 모니터의 입력 내용이 존재하면
//      HC12.write(Serial.read());      // 읽어서 HC-12 모듈로 전송
//    }
}
