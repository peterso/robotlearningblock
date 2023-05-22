# README

### Setup PC to read the task board state over USB:
1. Install dependencies
`python -m pip install pyserial`

2. Grant access rights to the USB Port (Linux)
`sudo chmod 666 /dev/ttyUSB0`

3. Plug in the M5Stick Controller to the PC with a USB cable and verify the device is connected
`lsusb`

4. Open a new terminal window and run the python script `read-serial.py`

### Quick Start
A python script for reading input from a USB serial port and saving the streamed data to a file.

Run python script.
`python scripts/read-serial.py`