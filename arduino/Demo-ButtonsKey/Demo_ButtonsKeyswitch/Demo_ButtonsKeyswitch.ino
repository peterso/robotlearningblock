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
int last_value_plug = 0;
int cur_value_plug = 0;

void setup() {
  // init lcd
  M5.begin();
  M5.Lcd.setRotation(3);
  pinMode(32, INPUT);         //GPIO32 (defined by Grove Port) for red button
  pinMode(33, INPUT);         //GPIO33 (defined by Grove Port) for blue button
  pinMode(26, INPUT_PULLUP);  //GPIO26 for the keyswitch input with pullup
  pinMode(0, INPUT_PULLUP);  //GPIO0 for the plug input with pullup
  pinMode(10, OUTPUT);        //GPIO10 the builtin LED
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setCursor(15, 2);
  M5.Lcd.println("Dual Button Key Demo");
  M5.Lcd.setTextColor(WHITE);
}

void loop() {
  cur_value_red = digitalRead(32);  //set red button to GPIO32
  cur_value_blue = digitalRead(33); //set blue button to GPIO33
  cur_value_key = digitalRead(26); //set keyswitch to GPIO26
  cur_value_plug = digitalRead(0); //set keyswitch to GPIO0

  M5.Lcd.setCursor(0,15); M5.Lcd.print("Blue Status: ");
  M5.Lcd.setCursor(0,30); M5.Lcd.print(" Red Status: ");
  M5.Lcd.setCursor(0,45); M5.Lcd.print(" Key Status: ");
  M5.Lcd.setCursor(0,60); M5.Lcd.print("Plug Status: ");

  // Check Blue Button
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

  // Check Red Button
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

  // Check Keyswitch
  if(cur_value_key != last_value_key){
    M5.Lcd.fillRect(75,45,100,15,BLACK);
    if(cur_value_key==0){
      M5.Lcd.setCursor(75,45); M5.Lcd.print("switched, 0");
      Serial.println("Key Status: key switched on");
      Serial.println(" value: 0");
    }
    else{
      M5.Lcd.setCursor(75,45); M5.Lcd.print("released, 1");
      Serial.println("Key Status: key released");
      Serial.println(" value: 1");
    }
    last_value_key = cur_value_key;
  }

  // Check plug
  if(cur_value_plug != last_value_plug){
    M5.Lcd.fillRect(75,60,100,15,BLACK);
    if(cur_value_plug==0){
      M5.Lcd.setCursor(75,60); M5.Lcd.print("seated, 0");
      Serial.println("Plug Status: Plug seated");
      Serial.println(" value: 0");
    }
    else{
      M5.Lcd.setCursor(75,60); M5.Lcd.print("removed, 1");
      Serial.println("Plug Status: Plug removed");
      Serial.println(" value: 1");
    }
    last_value_plug = cur_value_plug;
  }
  M5.update();
}
