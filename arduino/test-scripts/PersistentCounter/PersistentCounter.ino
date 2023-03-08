/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5StickC-Plus sample source code
*                          配套  M5StickC-Plus 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/m5stickc_plus
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/m5stickc_plus
*
* Describe: counter.  计数器
* Date: 2021/9/18
*******************************************************************************
*/

#include <M5StickCPlus.h>
#include <Preferences.h>

Preferences preferences;

int counter = 0;

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);  // Rotate the screen. 
    preferences.begin("my-app", false);  // We will open storage in RW-mode (second parameter has to be false
    counter = preferences.getUInt("counter", 0);  // Get the counter value in current namesapce, if no key exists then return default value as second parameter
    }

void loop() {
  M5.update();
  M5.Lcd.setCursor(0,0);
  M5.Lcd.printf("Current counter value: %d\n", counter);   // Print note to screen 
  if (M5.BtnA.isPressed()){
    resetCounter();
  }
  delay(2000);

  //update counter value
  counter++;
  preferences.putUInt("counter", counter);  // Store the counter to the Preferences namespace
  Serial.printf("counter value: %d\n", counter);
}

void resetCounter(){
    preferences.remove("counter");  // Or remove the counter key only.
    counter = preferences.getUInt("counter", 0);  // Get the counter value in current namesapce, if no key exists then return default value as second parameter
    Serial.printf("Counter value reset!\n");  // Print the counter to Serial Monitor. 
    M5.Lcd.printf("Counter value reset!\n");  // Print note to screen 
    delay(500);
    M5.Lcd.fillScreen(BLACK);
}
