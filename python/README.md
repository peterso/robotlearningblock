# README

### Setup PC to read from USB
1. Install dependencies
`python -m pip install pyserial`

2. Grant access rights to the USB Port
`sudo chmod 666 /dev/ttyUSB0`

### Quick Start
A python script for reading input from a USB serial port and saving the streamed data to a file.

Run python script.
`python scripts/read-serial.py`

### Button Box Protocol:
1. Press RED Button to start stopwatch. ==> stopwatch will begin counting on display
2. (Get key) and turn keyswitch. ==> LED will illuminate to indicate stopwatch can now be stopped
3. Press RED Button to stop stopwatch. ==> stopwatch will stop incrementing
4. Press BLUE Button to reset the stopwatch. ==> stopwatch value returns to 0.

The M5StickC device emits the current stopwatch face value in usecs and the following event messages:
- "Button Status: BtnA pressed"
- "Button Status: BtnA pressed STOP!" // emitted only after the stopwatch has been started with the first press
- "Button Status: BtnB pressed"
- "Button Status: Key switched!"

Note BtnA is the RED button and BtnB is the BLUE button.