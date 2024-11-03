# README Arduino Setup

The project is developed with the Arduino IDE.

If you want to flash the firware onto the M5StickPlus (ESP32) device then you will need to download the Arduino IDE and make the following changes to your configuration.

## Setup the Arduino IDE for the M5StickC

1. Add the M5Stack Library to your Library Path in Preferences [Github](https://docs.m5stack.com/en/quick_start/m5stickc_plus/arduino)
2. Install the "ESP32 Arduino" and "M5StickCPlus" Library using the Board Library Manager
3. Install the Wifi Manager Greeter "tzapu" using the Library Manager [Github](https://github.com/tzapu/WiFiManager#install-through-library-manager)

## How to flash new firmware onto your task board with Arduino IDE

1. Plug in the task board M5StickPlus unit to your PC with a USB cable.
2. Select the new device under the Port menu.
3. Select the board type as "M5StickC" under the Tools/Board menu.
4. Open the project folder containing the file you want to flash and compile and upload the .ino file. You should see an upload complete message in the command output section.
5. Verify the new firmware is working as expected by examining the serial output using the Serial Monitor tool.

## How to flash new firmware without the Arduino IDE
Use the flash-bin-no-ide.zip folder as a shareable object with the client. Compile the new target firmware with the Arduino IDE and then copy the *.bin files from your local PC into a new .zip folder.

The location of the *.bin files after compiling will be listed in the Arduino IDE in the terminal output when "verbose" flag for compiling is on in the Preferences menu. On a Mac, the location is something like: `/var/folders/bf/p280vkg91qj8wl5_1w4gn32h0000gn/T/arduino_build_168910/`

For Linux

Run the following two commands in a shell. Note, the second command parameters may need to be adapted depending on the file name during the binary export process and the port the task board controller is bound to.

`$ cd <PATH-to-UNZIPPED-FILES>`

`$ python esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 1500000 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xe000 ./boot_app0.bin 0x1000 ./bootloader_dio_80m.bin 0x10000 ./stopwatch_m5stickcplus.ino.bin 0x8000 ./stopwatch_m5stickcplus.ino.partitions.bin`

Tip: On Linux, if you get a permission denied error on /dev/ttyUSB0 try the following command:
`sudo chmod a+rw /dev/ttyUSB0`

## Helpful Coding Resources
- M5Stack LCD Library Documentation: https://github.com/m5stack/m5-docs/blob/master/docs/en/api/lcd.md 
- ESP32 Persistent Memory Example: https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/ 
