#include <HardwareSerial.h>
#include <Wire.h>

#define rxPin 25
#define txPin 26
#define SWITCH1 32

long baud = 9600;

HardwareSerial HC12(2);

void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  Serial.begin(baud); 
  HC12.begin(baud, SERIAL_8N1, rxPin, txPin); 
}

void loop() {
    while (HC12.available()) {        // HC-12에 수신 데이터가 존재하는 경우
    Serial.write(HC12.read());      // HC-12 모듈의 출력 내용을 읽어 시리얼 모니터로 전송
  }
  while (Serial.available()) {
//    sender();                         // 시리얼 모니터의 입력 내용이 존재하면
    HC12.write(Serial.read());      // 읽어서 HC-12 모듈로 전송
  }
//  if(digitalRead(SWITCH1) == HIGH) { 
//    Serial.println("SWITCH OFF"); 
//  }else{
//    Serial.println("SWITCH ON"); 
//  }
}
