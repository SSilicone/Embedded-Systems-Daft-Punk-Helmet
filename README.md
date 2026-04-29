# Embedded-Systems-Daft-Punk-Helmet
Final Project for Dr. Choi's ECE 428 Embedded Computer Systems. The objective is to implement two input peripherals and two output actuators into a working Embedded System.

I've decided to create a Daft Punk helmet that uses surrounding audio (preferably music) and interfaces it with a microcontroller that controls the output of the helmet's LED screen into a spectrogram or some other fun visual. It would also have an on board music player (most likely a homemade MP3) that sends music through a buffer for it to be modulated in real time (frequency, play speed, etc) and output on some affordable speaker.

### 2 Inputs:
- Surrounding audio into a microphone
- modulated SD card audio

### 2 Outputs:
- Helmet LED screen
- Modified music on peripheral speaker

### Notes:

If you plan to make your own Daft Punk helemt using similar materials, I highly recommend using [Gruthius](https://www.printables.com/model/299638-daft-punk-helmet) with their guide and stl files for a pretty swagger helment. They include cut outs in the ear cups for electroncis which makes wire routing very clean and efficient. 

## Real Time Spectrogram

### Microhpne:
This project uses a MAX4466, which is an analog microphone. To translate the analog signal into digital values, the ESP32 uses I2S with ADC mdoe to sample and conver the analog signal into discreeet voltage measurements. 

For this project, the I2S program measures 1,024 samples at a time, 40,000 times per second. This gives the ESP32 enough data to distinguish lower bass tones up to around 20,000Hz.

Notably, the microphone has a built in offest that shifts each sample up by a discreet voltage, which could cause phantom low frequency peaks. Subtracting by this offset before processing (and potentially a 10uF or lower capacitor at the output) gets rid of this issue. 

### FFT Processing: 
The ESP32 uses a Fast Fourier Transform to measure each set of 1024 samples and detrmine how much energy is in each frequency. This project uses something called a Hann window, which is a technique that smooths the beginninga and end of each sample set to zero to prevent aritificial frequency artifacts that would make the spectrogram look noisy. 

### LED Mapping: 
The frequency spectrum runs across the 32 pixel width of the matrix. The spectrum is mapped logarithmically along the matrix to prevent overcrowding in loewr frequencies and too narrow bins in the higher frequencies. The LED matrix is also mapped using the peak value in the FFT bin samples to make a sharper, more harmonic-based visualization. 

## Real Time Frequency Modulation

### Using a Pipeline
Rather than handling one giant task at a time, this project uses an audio pipeline that takes the audio between its origin, the resampler, and the eventual speaker ouptut. This is important to keep all tasks separate, since they all have separatea speeds for their individual tasks. It also helps with troubleshooting to determine where something is bottlenecking or encountering significant faults. 

This project uses FreeRTOS tasks and ques, where FreeRTOS is the operating system for the ESP32 and the queues are effectively the conveyors for the audio to travel along. 

The pipeline also uses a set of pre-allocated buffers (free pools). This prevents fragmentation from creating and destroying small memory blocks, and instead puts data in these pre-set buffers at startup, recycling them as they travel through the pipeline. 

### SD Card Reading:
The ESP32 has a continous running task that does nothing but read chunks of audio off of an SD card and depositing them in queues, looping after the audio ends. 

The audio file MUST be in a .wav format. Since the ESP32 is already strained from the copious amounts of FFT calculations, using an uncompressed file format prevents bottlenecking at the SD car reader task.

### Potentiometer Reading and Resampling:
The potentiometer allows you to change the playback speed from 0.5x to 2x the original auido. To do this, we need to read samples at a different rate than they were written. the resampler does this through interpolating between values, effectively making a linear best fit between data poitns to guess appropriate values. this avoides "stepping" of auido if it were to just ignore or repeat samples crudely. 

Along with the resampler, a dedicated task continously reads the potentiomer's analog value using an averaging filter to prevent jittering. It maps the 0 to 4095 raw ADC reading to a range of 0.5x to 2x speed. 

Crucially, the potentiometer reading is hooked to GPIO25 of the ESP32. This is to prevent conflict between the microphone's I2S ADC bank and the potentiometer ADC bank, because otherwise the I2S would take over and prevent the potetiometer readings. 

### Bluetooth Streaming: 
The ESP32 utilizes a callback function whenever the bluetooth needs more audio. It takes data chunks from the resampler, copies the samples into the form the libarary expects, and returns them. If the que runs out of data, it outputs silence rather than crashing.

Before starting bluetooth, the code pre-fills the resampled queue with several audio chunks to prevent drop-outs at the start of playback when the speaker connects. 

### Memory
I would like to note that external memory or PSRAM incorporated into the ESP32 is necessary to have enough space for the queues, resampler, bluetooth streaming, etc. Internal RAM is too small to hold all the audio buffers simultaneously, so extra RAM is needed to prevent crashing and allow smooth operation. 

## User Guide
1. Find your desired audio recording and create a .wav file format of it. Name it "audio.wav" for the code to recognize it.
2. Adjsut the program to recognize your specific bluetooth speaker. In the lat line of audio_pipeline.cpp change "J22" to your specific bluetooth device's name.
3. Begin the program. Adjus the potentiometer CW to go faster and CCW to go slower. 
