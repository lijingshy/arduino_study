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

## Arudino IDE support ESP8266 wifi
1. 安装扩展版
- 打开Arduino IDE找到首选项，添加附加开发板管理器网址http://arduino.esp8266.com/stable/package_esp8266com_index.json
- 打开开发板管理器中找到ESP8266下载即可，这时就可以选择开发板了
  
2. 代码范例
``` c++
#include <ESP8266WiFi.h>
...
```

