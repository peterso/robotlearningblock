# README Arduino Setup

The project is developed with the Arduino IDE.

If you want to flash the firware onto the M5StickPlus (ESP32) device then you will need to download the Arduino IDE and make the following changes to your configuration.

## Setup the Arduino IDE for the M5StickC

1. Add the M5Stack Library to your Library Path in Preferences [Github](https://docs.m5stack.com/en/quick_start/m5stickc_plus/arduino)
2. Install the "ESP32 Arduino" and "M5StickCPlus" Library using the Board Library Manager
3. Install the Wifi Manager Greeter "tzapu" using the Library Manager [Github](https://github.com/tzapu/WiFiManager#install-through-library-manager)

## How to flash new firmware onto your task board

1. Plug in the task board M5StickPlus unit to your PC with a USB cable.
2. Select the new device under the Port menu.
3. Select the board type as "M5StickC" under the Tools/Board menu.
4. Open the project folder containing the file you want to flash and compile and upload the .ino file. You should see an upload complete message in the command output section.
5. Verify the new firmware is working as expected by examining the serial output using the Serial Monitor tool.


