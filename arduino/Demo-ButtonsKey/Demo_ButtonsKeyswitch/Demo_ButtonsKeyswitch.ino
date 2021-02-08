/*
  This sketch is a demo of the button response time for a M5StickC unit with
  a DualButtonUnit from M5Stack and a keyswitch.
  author: Peter So peter.so@tum.de
*/
#include <M5StickC.h>

int last_value_red = 0;
int cur_value_red = 0;
int last_value_blue = 0;
int cur_value_blue = 0;
int last_value_key = 0;
int cur_value_key = 0;

void setup() {
  // init lcd
  M5.begin();
  M5.Lcd.setRotation(3);
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(26, INPUT_PULLUP); //GPIO26 for the keyswitch input
  pinMode(10, OUTPUT); //GPIO10 the builtin LED
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setCursor(15, 2);
  M5.Lcd.println("Dual Button example");
  M5.Lcd.setTextColor(WHITE);
}

void loop() {
  cur_value_red = digitalRead(32);  //set red button to GPIO32
  cur_value_blue = digitalRead(33); //set blue button to GPIO33
  cur_value_key = digitalRead(26); //set keyswitch to GPIO26

  M5.Lcd.setCursor(0,15); M5.Lcd.print("Blue Status: ");
  M5.Lcd.setCursor(0,30); M5.Lcd.print(" Red Status: ");
  M5.Lcd.setCursor(0,45); M5.Lcd.print(" Key Status: ");

  if(cur_value_blue != last_value_blue){
    M5.Lcd.fillRect(75,15,100,15,BLACK);
    if(cur_value_blue==0){
      M5.Lcd.setCursor(75,15); M5.Lcd.print("pressed, 0");
      Serial.println("Button Status: blue pressed");
      Serial.println(" value: 0");
    }
    else{
      M5.Lcd.setCursor(75,15); M5.Lcd.print("released, 1");
      Serial.println("Button Status: blue released");
      Serial.println(" value: 1");
    }
      last_value_blue = cur_value_blue;
  }

    if(cur_value_red != last_value_red){
      M5.Lcd.fillRect(75,30,100,15,BLACK);
      if(cur_value_red==0){
        M5.Lcd.setCursor(75,30); M5.Lcd.print("pressed, 0");
        Serial.println("Button Status: red pressed");
        Serial.println(" value: 0");
        digitalWrite(10, LOW); //turn on LED when red button is pressed
      }
      else{
        M5.Lcd.setCursor(75,30); M5.Lcd.print("released, 1");
        Serial.println("Button Status: red released");
        Serial.println(" value: 1");
        digitalWrite(10, HIGH); //turn off LED
      }
      last_value_red = cur_value_red;
  }

    if(cur_value_key != last_value_key){
    M5.Lcd.fillRect(75,45,100,15,BLACK);
    if(cur_value_key==0){
      M5.Lcd.setCursor(75,45); M5.Lcd.print("switched, 0");
      Serial.println("Key Status: key switched on");
      Serial.println(" value: 0");
    }
    else{
      M5.Lcd.setCursor(75,45); M5.Lcd.print("released, 1");
      Serial.println("Button Status: key released");
      Serial.println(" value: 1");
    }
      last_value_key = cur_value_key;
  }
  M5.update();
}
