#include <SoftwareSerial.h>
 
SoftwareSerial HC12(2,3); // HC-12 TX Pin은 D2(RX용)에, HC-12 RX Pin을 D3(TX용)

void setup() {
  Serial.begin(9600);             // 시리얼 모니터 속도 정의
  HC12.begin(9600);               // HC12 시리얼 통신 속도 정의

  // HC12 모듈 통신 테스트
  if (HC12.isListening()) {     
    Serial.println("OK");
  } else {
    Serial.println("NOK");
  }
}
 
void loop() {
  while (HC12.available()) {        // HC-12에 수신 데이터가 존재하는 경우
    Serial.write(HC12.read());      // HC-12 모듈의 출력 내용을 읽어 시리얼 모니터로 전송
//    Serial.print(HC12.read());
  }
}



//while (Serial.available()) {      // 시리얼 모니터의 입력 내용이 존재하면
//    HC12.write(Serial.read());      // 읽어서 HC-12 모듈로 전송
//  }
