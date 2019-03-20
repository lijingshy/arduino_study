# arduino_study

## ubuntu linux shell 
1. Install arduino and makefile system
``` sh
#download arduino and uncompress
#clone arduino makefile system
git clone https://github.com/sudar/Arduino-Makefile
#set environment
export ARDUINO_DIR=/home/lijing/tools/arduino-1.8.9
export ARDMK_DIR=/home/lijing/tools/Arduino-Makefile
export AVR_TOOLS_DIR=/home/lijing/tools/arduino-1.8.9/hardware/tools/avr
```
2. Test arduino compile system
``` sh
cd $ARDUINO_DIR/examples/01.Basics/Blink
#create Makefile and content

#USER_LIB_PATH=/home/lijing/sketchbook/libraries
#ARDUINO_LIBS=Servo
BOARD_TAG=uno
#MONITOR_PORT=/dev/ttyACM0
include $(ARDMK_DIR)/Arduino.mk

#compile 
make
#upload
make upload
```

