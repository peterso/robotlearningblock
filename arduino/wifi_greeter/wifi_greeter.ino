#include <M5StickCPlus.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

void setup() {
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

    // put your setup code here, to run once:
    Serial.begin(115200);
    
    // WiFi.mode(WiFi_STA); // it is a good practice to make sure your code sets wifi mode how you want it.

    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    //reset settings - wipe credentials for testing
    //wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }

    M5.begin(true, true, true); //screen, batt, serial

    digitalWrite(10, LOW); //turn on LED when red button is pressed
    delay(2000); //tmp delay just to verify setup...
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    
    Serial.begin(115200);
    M5.Imu.Init();
    
    // Lcd display setup
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
}

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

void loop() {
    // put your main code here, to run repeatedly:
    M5.Imu.getAccelData(&accX,&accY,&accZ);
    
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(0, 5);
    M5.Lcd.printf("Smart Task Board\n");
    M5.Lcd.printf("Wifi On:??? Status:%d\n", WiFi.status());
    M5.Lcd.printf("acX:%0.2f acY:%0.2f acZ:%0.2f\n", accX*1000, accY*1000, accZ*1000);
}
