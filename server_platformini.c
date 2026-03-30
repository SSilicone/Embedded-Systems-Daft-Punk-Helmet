; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600
upload_port = /dev/ttyUSB0
monitor_port = /dev/ttyUSB0
monitor_filters = esp32_exception_decoder
board_build.partitions = huge_app.csv
lib_deps =
    fastled/FastLED @ ^3.6.0
    kosme/arduinoFFT @ ^2.0.1   
     https://github.com/pschatzmann/ESP32-A2DP.git#v1.8.4
build_flags =
    -DCORE_DEBUG_LEVEL=0  
    -DCONFIG_BT_ENABLED
    -DCONFIG_CLASSIC_BT_ENABLED
    -DCONFIG_A2DP_ENABLE
     -DCONFIG_BTDM_CTRL_MODE_BLE_ONLY=0
    -DCONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=0
    -DCONFIG_BTDM_CTRL_MODE_BTDM=1
    -DCONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM=4
    -DCONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM=8
    -DCONFIG_ESP32_WIFI_TX_BUFFER=DYNAMIC
build_unflags =
    -DCONFIG_BT_BLE_ENABLED
