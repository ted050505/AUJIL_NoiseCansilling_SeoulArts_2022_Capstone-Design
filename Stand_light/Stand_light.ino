#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN        6
#define NUMPIXELS 3 // Popular NeoPixel ring size

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

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
}

void loop() {
  unsigned long currentMillis = millis();

  while (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);

    // 헤드폰 미 착용 시 동작
    if (c == '1') {
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
    // 헤드폰 착용 시 동작
    if (c == '2') {
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
    // 헤드폰 4명 착용 시 동작
    if (c == '3') {
      pixels.clear();
      for (int u = 0; u < 255; u++) {
        for (int i = 0; i < 3; i작++) {
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
