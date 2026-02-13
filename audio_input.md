# Board and Platform
I decided to use an ESP32 board for the audio processing side. For my code, I utlized VScode with extensiosn PlatformIO-IDE and ESP-IDF for porting to the ESP32. To generate plots of audio data versus just printed voltage readings,
i also used the teleport extension on Vscode for serial plotting.






## Processing Input Audio
For this project, I will be taking a lot of inspiration from [atomic14](https://github.com/atomic14/esp32_audio),since he provides a very neat walkthrough on analog-to-digital processing and filtering.   
ool
For additional audio resources, I highly recommend using the [arduino audio tools](https://github.com/pschatzmann/arduino-audio-tools) github given their ample library of DSP tools.
