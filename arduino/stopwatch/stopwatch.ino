#include <M5StickC.h>

#define LCDHIGH 240
#define LCDWIDTH 320

#define BTN_RED 32 // Dual Button Module - Red Button
#define BTN_BLU 33 // Dual Button Module - Blue Button
#define KEYSWITCH 26 // Keyswitch
#define PLUG 0 // Plug
#define TIMELIMIT 30  // Trial Time Limit in seconds

#define BUTTON_ON 0
#define BUTTON_OFF 1

//timer interrupt variable.
volatile unsigned long usecCount = 0;
hw_timer_t *interrupptTimer = NULL;
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

//min,sec,msec,usec display.
int display[4] = {0};

//timer start/stop check variable
int countStart = 0;
int started = 0;
int keyswitch = 0;
int plug = 0;
int timeLeft = 0;

void IRAM_ATTR usecTimer()
{
  portENTER_CRITICAL_ISR(&mutex);
  usecCount += 5;
  portEXIT_CRITICAL_ISR(&mutex);
}

void setup()
{
  // initialize the M5Stack object
  M5.begin();

  //GPIO setting  
  pinMode(BTN_RED, INPUT);          //Button A = GPIO 39
  pinMode(BTN_BLU, INPUT);          //Button B = GPIO 38
  pinMode(KEYSWITCH, INPUT_PULLUP); //Button C = GPIO 26
  pinMode(PLUG, INPUT_PULLUP);      //Button D = GPIO 0
  pinMode(10, OUTPUT);              //GPIO10 the builtin LED

  // Lcd display setup
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  //M5.Lcd.setCursor(0, LCDWIDTH / 4);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf(" m: s: ms: us\n");
  M5.Lcd.printf("00:00:000:000\n");

  //interrupt timer setting
  //timerBegin is count per 100 microsec.
  interrupptTimer = timerBegin(0, 80, true);
  //interrupt method setting
  timerAttachInterrupt(interrupptTimer, &usecTimer, true);
  //interrupt timing setting.
  timerAlarmWrite(interrupptTimer, 5, true);
  timerAlarmDisable(interrupptTimer);
  //timerAlarmEnable(interupptTimer);
  digitalWrite(10, LOW); //turn on LED when red button is pressed
  delay(2000); //tmp delay just to verify setup...
  digitalWrite(10, HIGH); //turn off LED when red button is pressed
}

void loop()
{
  // put your main code here, to run repeatedly:

  //Start Button Check
  if (digitalRead(BTN_RED) != BUTTON_OFF && started == 0 && digitalRead(KEYSWITCH) == 1 && digitalRead(PLUG) == 1)
  {
    delay(1);
    if (digitalRead(BTN_RED) != BUTTON_OFF)
      countStart = 1;
      digitalWrite(10, HIGH); //turn off LED
      Serial.println("Button Status: BtnA pressed");

    for (;;)
      if (digitalRead(BTN_RED) == BUTTON_OFF)
        break;
    delay(1);
  }

  //Stop Button Check
  if (digitalRead(BTN_RED) != BUTTON_OFF && started == 1 && plug == 1 && keyswitch == 1)
  {
    delay(1);
    if (digitalRead(BTN_RED) != BUTTON_OFF)
      countStart = 0;
      Serial.println("Button Status: BtnA pressed STOP!");

    for (;;)
      if (digitalRead(BTN_RED) == BUTTON_OFF)
        break;
    delay(1);
  }

  //Time Limit Check
  if (started == 1 && timeLeft <= 0)
  {
    delay(1);
      countStart = 0;
      Serial.print("Time's Up! Trial Time Limit: ");
      Serial.println(TIMELIMIT);
    delay(1);
  }

  //Keyswith Check
  if (digitalRead(KEYSWITCH) != BUTTON_OFF && started == 1 && keyswitch == 0)
  {
    delay(1);
    keyswitch = 1;
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Button Status: Key switched!");
  }

  //Plug Check
  if (digitalRead(PLUG) != BUTTON_OFF && started == 1 && plug == 0)
  {
    delay(1);
    plug = 1;
    digitalWrite(10, HIGH); //turn on LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Button Status: plug seated!");
  }

  //Time Count  Start
  if (countStart == 1 && started == 0)
  {
    timerAlarmEnable(interrupptTimer);
    started = 1;
  }

  //Time Count  Stop
  if (countStart == 0 && started == 1)
  {
    timerAlarmDisable(interrupptTimer);
    started = 0;
    keyswitch = 0;
    plug = 0;
  }

  //Count Reset Check
  if (digitalRead(BTN_BLU) != BUTTON_OFF && started == 0)
  {
    delay(1);
    if (digitalRead(BTN_BLU) != BUTTON_OFF)
      Serial.println("Button Status: BtnB pressed");

      usecCount = 0;
      digitalWrite(10, HIGH); //turn off LED

    for (;;)
      if (digitalRead(BTN_BLU) == BUTTON_OFF)
        break;
    delay(1);
  }

  //time calculation
  display[3] = (int)(usecCount % 1000);
  display[2] = (int)((usecCount % 1000000) / 1000);
  display[1] = (int)((usecCount / 1000000) % 60);
  display[0] = (int)((usecCount / 60000000) % 3600);
 
  //count display
  //portENTER_CRITICAL(&mutex);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  //M5.Lcd.setCursor(0, LCDWIDTH / 4);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf(" m: s: ms: us\n");
  M5.Lcd.printf("%02d:",display[0]);
  M5.Lcd.printf("%02d:",display[1]);
  M5.Lcd.printf("%03d:",display[2]);
  M5.Lcd.printf("%03d\n",display[3]);
  M5.Lcd.printf("Thing:set/pin\n");
  M5.Lcd.printf("K:%d/%d P:%d/%d\n", keyswitch, digitalRead(KEYSWITCH), plug, digitalRead(PLUG));
  //M5.Lcd.printf("%lu", usecCount);
  M5.Lcd.printf("%d", timeLeft);
  Serial.println(usecCount); //print out seconds to the serial monitor
  delay(5); // delay for screen refresh NOTE: This directly affects performance of clock buttons
  //portEXIT_CRITICAL(&mutex);
  
  timeLeft = TIMELIMIT - display[1];
}
