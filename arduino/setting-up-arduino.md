# A Beginner's Guide to setting up the Arduino IDE with the M5StickC Development Board

This guide explains how to setup the Arduino IDE on a development PC in order to program a new task board microcontroller from M5Stack.

## Instructions for setting up on Linux Ubuntu
1. Download the zip file of the arduino ide from the arduino website
https://www.arduino.cc/en/guide/linux
2. Unzip the file
3. Open the arduino IDE by running the arduino.sh file in a terminal
4. Add the Espressif URL to the Board Manager list in Preferences (Arduino IDE/Preferences/Settings/Additional boards manager URLs):
    https://dl.espressif.com/dl/package_esp32_index.json
5. Add the board files for ESP32 using the Board Manager
6. Add the libraries for ESP32 and M5StickC
7. Install pyserial with pip3. (Optional)

## Verify your Setup
Plug in the M5StickC board and see if it is recognized by the computer. It should be viewable in the arduino IDE under Mene/Tools/Port as /dev/cu.usbserial.###### on MacOS or /dev/ttyUSB# on Linux.

Before uploading to the board, confirm the following settings for the board target, port, and baud rate in the IDE under the Menu/Tools:
- Board = M5StickC, 
- Port = /dev/ttyUSB0, (or similar) 
- Baud = 1500000

Note that each task board controller should have a unique Token in order to be recognized by Kaa. Tokens have the general form of `task_board_###` where ### refers to the number on the sticker applied to the side of the task board. The token is specified in the firmware in the `secrets.h` file as variable `SECRET_TOKEN`.

If you are creating a new task board device you will need to generate a new device and token on the KaaIoT platform. You will need sufficient permissions from the KaaIoT account (member of the 'Dev Team') to generate new tokens. To generate a new token go to the [Kaa Device Management Portal](https://cloud.kaaiot.com/devices/device-management/), select the correct Application, this project is currently using "smart task board proto" with App Version Name="c1v9jqmgul2l1s47m6bg-v0" and generate a new token by clickgin on "Add device" in the top right of the window. 

The current subscription with Kaa is limited to ~~15~~, ~~50~~, 100 devices. 

## Flashing new firmware to the task board microcontroller
After verifying all the steps above you can flash the new firmware to the microcontroller with the "upload" command.

Wait for the upload to take place (~1min). When the upload is finished, you should see the message upload complete in the IDE and the screen on the M5StickC unit should restart.
