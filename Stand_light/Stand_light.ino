#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN        6
#define PIN1        7
#define PIN2        8
#define PIN3        9

#define NUMPIXELS 3 // Popular NeoPixel ring size

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels1(NUMPIXELS, PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(NUMPIXELS, PIN2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels3(NUMPIXELS, PIN3, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500

int previousMillis = 0;
int interval = 5;

void setup() {
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  Serial.begin(9600);
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(255);
  pixels1.begin();
  pixels1.clear();
  pixels1.setBrightness(255);
  pixels2.begin();
  pixels2.clear();
  pixels2.setBrightness(255);
  pixels3.begin();
  pixels3.clear();
  pixels3.setBrightness(255);
}

void loop() {
  unsigned long currentMillis = millis();

  while (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);

    // 헤드폰1 미 착용 시 동작
    if (c == '0') {
      pixels.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels.setBrightness(u);
            pixels.setPixelColor(i, pixels.Color(255, 130, 30));
            pixels.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰1 착용 시 동작
    if (c == '1') {
      pixels.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;
            
            pixels.setBrightness(u);
            pixels.setPixelColor(i, pixels.Color(50, 255, 50));
            pixels.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰2 미 착용 시 동작
    if (c == '2') {
      pixels1.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels1.setBrightness(u);
            pixels1.setPixelColor(i, pixels1.Color(255, 130, 30));
            pixels1.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰2 미 착용 시 동작
    if (c == '3') {
      pixels1.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels1.setBrightness(u);
            pixels1.setPixelColor(i, pixels.Color(50, 255, 50));
            pixels1.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰2 미 착용 시 동작
    if (c == '2') {
      pixels1.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels1.setBrightness(u);
            pixels1.setPixelColor(i, pixels1.Color(255, 130, 30));
            pixels1.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰2 미 착용 시 동작
    if (c == '3') {
      pixels1.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels1.setBrightness(u);
            pixels1.setPixelColor(i, pixels.Color(50, 255, 50));
            pixels1.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰3 미 착용 시 동작
    if (c == '4') {
      pixels2.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels2.setBrightness(u);
            pixels2.setPixelColor(i, pixels1.Color(255, 130, 30));
            pixels2.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰3 미 착용 시 동작
    if (c == '5') {
      pixels2.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels2.setBrightness(u);
            pixels2.setPixelColor(i, pixels.Color(50, 255, 50));
            pixels2.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰3 미 착용 시 동작
    if (c == '4') {
      pixels2.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels2.setBrightness(u);
            pixels2.setPixelColor(i, pixels1.Color(255, 130, 30));
            pixels2.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰3 미 착용 시 동작
    if (c == '5') {
      pixels2.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels2.setBrightness(u);
            pixels2.setPixelColor(i, pixels.Color(50, 255, 50));
            pixels2.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰4 미 착용 시 동작
    if (c == '6') {
      pixels3.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels3.setBrightness(u);
            pixels3.setPixelColor(i, pixels1.Color(255, 130, 30));
            pixels3.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    // 헤드폰4 미 착용 시 동작
    if (c == '7') {
      pixels3.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            pixels3.setBrightness(u);
            pixels3.setPixelColor(i, pixels.Color(50, 255, 50));
            pixels3.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
    
    // 헤드폰 4명 착용 시 동작
    if (c == '9') {
      pixels.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i++) {
          if (currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;
            
            pixels.setBrightness(u);
            pixels.setPixelColor(i, pixels.Color(255, 20, 20));
            pixels.show();   // Send the updated pixel colors to the hardware.
          }
        }
      }
    }
  }
}
