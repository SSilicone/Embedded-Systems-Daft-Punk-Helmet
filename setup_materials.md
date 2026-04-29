# Board and Platform
I decided to use an ESP32 WROVER board with 8MB PSRAM for the audio processing side. For my code, I utlized VScode with extensiosn PlatformIO-IDE and ESP-IDF for porting to the ESP32. The code is written in C and the finished code files are labled with "FIN" in the GitHub. 



## Tools for Processing Input Audio
For this project, I will be taking a lot of inspiration from [atomic14](https://github.com/atomic14/esp32_audio),since he provides a very neat walkthrough on analog-to-digital processing and filtering. 

For additional audio resources, I highly recommend using the [arduino audio tools](https://github.com/pschatzmann/arduino-audio-tools) github given their ample library of DSP tools.

For troubleshooting and generla coding questions, I recommend a moedestly shameful amount of vibe coding (only with Claude though).



## Materials Used
The list of materials to date are gvien here. some itmes will be marked with an asterik. These asteriks indicate that the item is not specific, and falls within a family of suitable compoenents that are free to choose without code discrepancy. 

- WS2812B LED 8x32 Matrix
- ESP32 WROVER devkit (at LEAST 4MB of PSRAM recommended)
- *jumper wires M-M and M-F
- *10k Pot Ohm
- *Bluetoooth Speaker
- Breadboard/Proto PCB
- SD card reader module
- SD card (recoomended >= 1GB

