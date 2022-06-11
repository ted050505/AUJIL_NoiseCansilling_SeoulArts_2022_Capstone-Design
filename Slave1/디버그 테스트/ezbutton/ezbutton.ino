#define SWITCH1 32

void setup() {
  pinMode(SWITCH1, INPUT_PULLUP);
  Serial.begin(9600);
} 

void loop() { 
  if(digitalRead(SWITCH1) == !HIGH) { 
    Serial.println("SWITCH ON"); 
  }else{
    Serial.println("SWITCH OFF"); 
  }
}
