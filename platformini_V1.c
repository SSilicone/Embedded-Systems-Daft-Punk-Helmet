[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino

monitor_speed  = 115200
upload_speed   = 921600
upload_port    = /dev/ttyUSB0
monitor_port   = /dev/ttyUSB0
monitor_filters = esp32_exception_decoder

board_build.partitions = huge_app.csv

; ★ Enable PSRAM support — required for ps_malloc() to work
board_build.arduino.memory_type = qio_qspi
build_flags =
    -DCORE_DEBUG_LEVEL=4
    -DCONFIG_BT_ENABLED
    -DCONFIG_CLASSIC_BT_ENABLED
    -DCONFIG_A2DP_ENABLE

build_unflags =
    -DCONFIG_BT_BLE_ENABLED

lib_deps =
    fastled/FastLED @ ^3.6.0
    kosme/arduinoFFT @ ^2.0.1
    https://github.com/pschatzmann/ESP32-A2DP.git#v1.8.4
