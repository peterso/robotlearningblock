For Linux

Run the following two commands in a shell and change the parameter in the second command to match the number on the device sticker label, i.e. ./iot-task-board-m5stickcplus-146.ino.bin to ./iot-task-board-m5stickcplus-147.ino.bin, etc.

$ cd <PATH-to-UNZIPPED-FILES>
$ python esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 1500000 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xe000 ./boot_app0.bin 0x1000 ./bootloader_dio_80m.bin 0x10000 ./iot-task-board-m5stickcplus-146.ino.bin 0x8000 ./stopwatch_m5stickcplus.ino.partitions.bin

Tip: if you get a permission denied on /dev/ttyUSB0 try the following command:
`sudo chmod a+rw /dev/ttyUSB0`
