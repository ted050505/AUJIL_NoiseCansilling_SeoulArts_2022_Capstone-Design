# Aujil_NoiseCansilling_Project

<img src="https://user-images.githubusercontent.com/83487028/187818910-06db9ec6-8bce-4bfb-887f-6429602ce8b9.jpeg" width="1280px" height="720px" title="100px" alt=""></img>
<br>
Gen : Interactive Installtion


# 하드웨어 개발
 + 헤드폰
   + 노이즈 캔슬링을 체험 할 수 있는 헤드폰으로, 인터렉션 요소가 된다.
 + 송/수신 개발보드
   + 인터렉션을 위해 센서 값을 보정하는 계산 과정과 이를 토대로 안정화 된 센서 값을 RF통신으로 송, 수신하는 역할을 한다.


# 창작 과정 및 방법
+ 하드웨어 시스템 설계

  + 인터렉션
    작품은 헤드폰 착용을 통해 직접적인 콘텐츠 인터렉션이 동작한다. 헤드폰 스탠드에 FSR (Force Sensitive Sensor) 압력센서와 같이 상태 값을 센싱 할 수 있는 센서로 헤드폰이 스탠드에서 떨어지는 것
    을 통해 간접적으로 착용을 감지하는 방식과 헤드폰에 리밋 스위치 또는 적외선 센서 부착을 통해 착용을 직접적으로 감지하는 방식으로 두 가지의 하드웨어 시스템을 생각해 볼 수 있었다. 
    
    작품에서는 헤드폰에 부착된 6축 IMU 센서를 통해 센싱하고 공간음향을 구현해야 하는데, 스탠드에 센서를 부착하여 간접적으로 착용을 감지하는 방식을 채택하게 될 경우, 헤드폰과 헤드폰 스탠드에 다수의 개발
    보드가 필요하기에 제한된 제작비를 생각 했을 때 상당히 비효율적인 구조였다. 때문에 최소한의 제작 비용과 통일된 하드웨어 구조를 위해 헤드폰에 리밋 스위치 부착을 통해 착용을 직접적으로 감지하는 방식의 
    하드웨어 시스템을 채택했다.
    
    작품에서는 센서들의 동작을 디스플레이로 확인하고, 사용하는 모듈을 최소화 하기 위해 TTGO T-display 보드를 사용했다. 보드를 헤드폰에 부착 시 경량화, 전력 소비, 상호 운용성, 성능 측면에서 설계 최
    적화 위해 Lolin D32 PRO V2, Arduino nano 33 iot, TTGO T-Display 3가지 보드 검토 및 테스트 과정을 거쳤다. 선택한 TTGO 개발보드는 충전 모듈, 과충전 방지 모듈이 패키징 되어 있기에 
    부착하는 모듈을 최소화 할 수 있었다.

  + 센서
    헤드폰에 사용된 센서는 착용을 감지하는 접촉식 센서인 리밋 스위치와 헤드폰의 각도를 계산해 줄 6축 IMU 센서다. 비 접촉식 센서인 조도 센서와 적외선 센서를 고안하였지만, 조도센서의 경우 전시 환경에 따
    라 데이터 값이 좌우 되는 등 문제점을 발견하여 리밋 스위치를 사용했다.

  + 통신
    작품의 마감 완성도, 연출을 위해 Serial 무선 통신을 사용했다. 사용할 수 있는 무선 통신 방법은 블루투스, 와이파이, RF 통신으로 3가지로 볼 수 있다. 헤드폰에 부착된 센서, 개발보드를 배터리를 통해   
    동작 시켜야 했기에 전력소모에 민감해야했다. 이와 같은 이유로 저전력인 BLE bluetooth 5.0 통신을 통해 데이터를 송/수신하는 방법을 설계 해보았으나, 통신 거리, 주파수 간섭 현상 문제가 있었다. 해당 
    블루투스 통신의 경우 통신 거리가 5~10m 내외로 짧다는 점과 2.4GHz ISM 스펙트럼 대역(2400~2483.5MHz) 사용해야 하기 떄문에 전시 현장에서 동일 채널을 사용하는 다른 무선 장치 또는 블루투스간의 
    간섭 현상 우려가 있을 수 밖에 없었다. Wifi 장치도 2.4GHz ISM 스펙트럼 대역 사용하고 데스크탑의 USB 3.0 에서도 2.4Ghz 주파수 대역의 전자파를 발생 시켜 신호 간섭 현상을 발생시키는 문제 중 하
    나이다.
    
    ![Uploading 화면 캡처 2022-07-08 082045.png…]()
    < Spectrum Analyzer 로 2400~2480Ghz 주파수 대역을 측정한 것 >
    
    위 사진을 통해 볼 수 있듯이, 2400~2480Ghz 대역은 낮은 주파수 대역에 비해 RF 밀도가 높아 비슷한 채널은 신호 간섭이 일어날 수 있는 것을 확인 할 수 있다. 5.0 블루투스로 넘어와도 
    거리 문제와 간섭 현상을 최소화하기 위해 블루투스 대안으로 433Mhz 대역의 RF 통신 모듈인 HC-12 (433.4MHz to 473.0MHz) 를 사용했다.


  + 전원
    엑추에이터를 사용하지 않아 많은 양의 전력을 필요로 하지 않고, 리튬 이온 배터리의 경우 전해질이 액체이며, 액체 전해질의 경우 폭발 위험성이 있으나, 리튬 폴리머 배터리의 경우 전해질이 반고체 상태의 젤
    을 사용하여 폭발 위험성 적기 때문에 3.7V 리튬 폴리머 배터리 500MA 를 채결하는 방식을 사용했다. 

  + 최종 하드웨어 시스템 설계도 
  
    < 기존 하드웨어 시스템 구조 다이어그램 >
    
    총 4대의 헤드폰을 사용하기 때문에 각 헤드폰마다 송신 역할을 하는 RF 통신 모듈 (HC-12)를 부착하고 4대의 헤드폰마다 송신하는 데이터를 받기 위해, 모든 송신기 모듈의 채널과 수신기 모듈 채널을 동일하
    게 설정하여 1대의 RF 통신 모듈로 데이터를 수신하도록 설정했다. 하지만 이와 같이 설정 후 데이터를 수신 시 한 시리얼 포트를 통해서만 버퍼에 데이터를 저장 후 출력하기 때문에, 3대 이상의 헤드폰 데이
    터는 무시하는 문제가 발생했다. 이와 같이 데이터를 무시하는 현상을 대처하기 위한 방법은 각 헤드폰 RF 통신 모듈의 채널을 달리하고 수신부 RF 통신 모듈을 추가하여 1대1 시리얼 통신으로 변경하는 것이
    다.
    
    < 변경된 하드웨어 시스템 구조 다이어그램 >
    
    각각의 헤드폰마다 RF 통신 모듈의 채널 값을 달리하여 설정 후, 아두이노 보드와 RF 통신 모듈 4개를 사용하여 9600bps 속도로 1대1 시리얼 통신이 가능하게 했다. 



+ 하드웨어 프로그래밍

  + 송신부 하드웨어 프로그램
    송신부 하드웨어 프로그램은 리밋 스위치가 눌리면 6축 IMU 센서의 데이터 값이 RF 통신 모듈로 송신하는 전체 구조를 가지고 있다. 
    
    < 송신부 하드웨어 메인 프로그램 소스 코드 >
    
    가속도 센서는 가만히 있을 때 센서에 작용하는 중력가속도를 X, Y, Z 축으로 벡터 3개로 나누어 크기를 측정해 준다. 진동과 외력(이동)에 측정값이 왜곡될 수 있지만 시간이 지나도 오차에 강한 특징을 가진
    다. 이리저리 자세가 변하더라도 노이즈가 끼지만 원래 자세로 돌아오면 이전에 출력한 값과 똑같은 값이 나온다.
    
    자이로 센서는 센서의 회전이 발생하면 XYZ 축의 각속도 변화량을 측정해준다. 그런데 각속도이기 때문에 각도(위치)를 구하려면 적분을 해줘야 한다. 적분을 하는 과정에서 센서의 노이즈도 같이 적분되기 때문
    에 누적오차가 커지게 된다. 결론적으로 자이로 센서는 실제 움직임과 비슷한 값(정확한 값)을 출력하지만 누적오차가 발생한다.
   
    MPU6050 으로 기본적으로 측정하는 데이터는 누적오차와 같은 잡음이 생기기 때문에, 더 정확하고 안정된 데이터 값을 얻기위해 가속도, 자이로 센서에 칼만 필터를 적용했다. 칼만 필터는 과거와 현재 데이터
    값을 기준으로 재귀적 연산을 통해 최적값을 예측하는 것이다.
    
    직렬 포트로 데이터를 송/수신하는 RF 통신 모듈을 사용하기 위해서는 개발보드와 시리얼 통신을 해주어야 한다. Esp32 기반인 TTGO 보드에서 소프트웨어 시리얼을 지원하지 않기 때문에 하드웨어 시리얼을 통
    해 RF 통신 모듈과 시리얼 통신을 했다.
    
    TTGO 보드에서는 25번 핀과 26번 핀이 하드웨어 시리얼을 지원한다.
    
    RF 모듈을 사용하기 위해서는 프로그램의 초기 설정을 해주듯이 RF Module AT Commands 통신을 통해 모듈의 채널, 통신 감도, 송신 전력과 같은 초기 설정을 해주어야 하고 송신기와 수신기의 설정을 동일
    하게 설정하여 데이터를 주고 받을 수 있도록 했다.










  + 수신부 하드웨어 프로그램
    
    수신부 하드웨어 프로그램은 RF 통신 모듈로부터 데이터를 받아오는 구조를 가지고 있다.
    
    [ 수신부 하드웨어 메인 프로그램 소스 코드 ] 
    
    데이터를 수신하는 보드의 프로그램으로 소프트웨어 시리얼 보다 보드 내에서 처리하는 과정이 적어 더 빠른 강점을 지닌 하드웨어 시리얼을 통해 수신하도록 했다. 송신을 하여 헤드폰 보드를 제어할 필요는 없기
    에 수신 코드만 작성했다.




 + 디스플레이 UI 디자인 (TTGO T-Display 1.14 inch UI 디자인)
   
   전시 중 실시간 하드웨어 확인을 위해 총 디스플레이의 3가지 인터페이스 구현했다.
   로고가 나오는 메인화면, 실시간 센서 값/배터리 잔량 확인 화면, 배터리 소모를 방지하고자 유휴 모드를 설정 할 수 있다.
   
   디스플레이에 이미지를 업로드 하기 위해서는 이미지 파일을 업로드 하는 것이 아닌, 각 픽셀들을 16진수로 표현되는 HEX 코드로 변환하여 코드를 업로드 하는 방식을 사용한다.
   T-Display LCD 1.14 inch 보드가 제공하는 디스플레이 해상도는 240x135픽셀이다. 16:9 비율로 이미지를 줄이거나 같은 비율로 이미지를 만들어 줄이는 과정과 이미지를 헥스 코드로 변환하는 과정을 거
   쳐 업로드 파일에 작성 해주면 된다.


< 이미지 사이즈 변환 프로그램 >

< *이미지 Hex code 변환 프로그램, 헤더파일에 입력한 Hex 코드 > 
이미지 첨부 *https://creator-es.tistory.com/entry/12331



+ 하드웨어 제작

완성도를 위해 헤드폰 안에 배터리와 6축 IMU 센서, 리밋 스위치를 넣고 봉합하였다. 리밋 스위치를 통해 착용 여부를 감지하고 가속도-자이로 센서의 Roll, Pitch 데이터 값으로 실시간 공간 음향을 구현한다.

헤드폰 안에 넣고 봉합 할 경우 배터리, 센서, 단선을 숨길 수 있어서 마감 완성도가 높아진다. 대신, 정확한 센싱을 위해 리밋 스위치의 위치와 고정이 필요하고, 착용감에 불편이 없도록 하는 것이 관건이다.


< 3.7V 배터리, Limit 스위치, IMU 센서 봉합 >




실제 전시 공간인 302호의 전파를 측정하고 간섭이 일어날 수 있는 환경일 경우 RF 통신 모듈의 채널 변경이 필요했기에, Spectrum Analyzer로 400~500mhz 주파수 대역을 측정했다. 


< Spectrum Analyzer 로 400~500mhz 주파수 대역 측정 >



