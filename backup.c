#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <FastLED.h>

// --- Configuration Constants ---
#define I2S_PORT I2S_NUM_0
#define ADC_CHANNEL ADC1_CHANNEL_7 // GPIO35
#define SAMPLE_RATE 20480          // 20.48 kHz ensures good coverage up to ~10kHz
#define SAMPLES 512                // Must be a power of 2
#define MATRIX_WIDTH 32
#define MATRIX_HEIGHT 8
#define LED_PIN 18                 // Change to your actual FastLED data pin
#define NUM_LEDS (MATRIX_WIDTH * MATRIX_HEIGHT)

// --- FastLED Setup ---
CRGB leds[NUM_LEDS];

// --- FFT Setup ---
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_RATE);

//Gloab varaibles
float peakHeights[MATRIX_WIDTH] = {0}; // Stores the current visual height
float decayRate = 0.2;                 // How fast the bars fall (0.1 to 0.5)
int noiseThreshold = 800;              // Adjust this based on your room's silence

// Shared array to pass amplitude data between Core 0 and Core 1
// volatile ensures the compiler doesn't optimize away cross-core updates
volatile uint8_t columnAmplitudes[MATRIX_WIDTH] = {0}; 

// Task handles
TaskHandle_t AudioTaskHandle;

// --- I2S Configuration ---
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  // Install and start I2S driver
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_adc_enable(I2S_PORT);
}

// --- Core 0: Audio Sampling & FFT Task ---
void audioTask(void *pvParameters) {
  size_t bytesRead;
  uint16_t i2sBuffer[SAMPLES];

  for(;;) {
    i2s_read(I2S_PORT, &i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

    // Format and Remove DC Offset
    double mean = 0;
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = i2sBuffer[i];
        vImag[i] = 0.0;
        mean += vReal[i];
    }
    mean /= SAMPLES;
    for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;

    // Process FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    /* LOGARITHMIC MAPPING LOGIC 
       We use a power function to determine which FFT bins go into which column.
    */
    for (int col = 0; col < MATRIX_WIDTH; col++) {
      // Calculate start and end bins for this column using a power curve
      // This groups more bins together as we move toward the high frequencies
      int binStart = pow(1.15, col); 
      int binEnd = pow(1.15, col + 1);
      
      // Ensure we stay within the FFT result bounds (SAMPLES/2)
      if (binEnd > SAMPLES / 2) binEnd = SAMPLES / 2;
      if (binStart >= binEnd) binEnd = binStart + 1;

      float averageAmplitude = 0;
      for (int i = binStart; i < binEnd; i++) {
        averageAmplitude += vReal[i];
      }
      averageAmplitude /= (binEnd - binStart);

      // 1. Noise Gate
      if (averageAmplitude < noiseThreshold) averageAmplitude = 0;

      // 2. Scale to Matrix Height (0-8)
      // Tweak the '1500' divisor to change sensitivity
      float targetHeight = averageAmplitude / 1500.0;
      if (targetHeight > MATRIX_HEIGHT) targetHeight = MATRIX_HEIGHT;

      // 3. Temporal Smoothing (Decay/Gravity)
      if (targetHeight >= peakHeights[col]) {
        // If the new sound is higher, jump to it immediately
        peakHeights[col] = targetHeight;
      } else {
        // If the sound is lower, let the bar "fall" slowly
        peakHeights[col] -= decayRate;
        if (peakHeights[col] < 0) peakHeights[col] = 0;
      }

      columnAmplitudes[col] = (uint8_t)peakHeights[col];
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

// --- Setup (Runs on Core 1) ---
void setup() {
  Serial.begin(115200);
  
  // Setup FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50); // Keep it low while testing to avoid power spikes
  FastLED.clear();
  FastLED.show();

  setupI2S();

  // Pin the audio task to Core 0. 
  // Loop() automatically runs on Core 1.
  xTaskCreatePinnedToCore(
    audioTask,       // Function to implement the task
    "Audio_FFT",     // Name of the task
    10000,           // Stack size in words
    NULL,            // Task input parameter
    1,               // Priority of the task
    &AudioTaskHandle,// Task handle
    0                // Core where the task should run
  );
}

// --- Core 1: FastLED Display Task ---
void loop() {
  FastLED.clear();

  // Draw the matrix based on the shared amplitude array
  for (int x = 0; x < MATRIX_WIDTH; x++) {
    int height = columnAmplitudes[x];
    
    for (int y = 0; y < height; y++) {
      // WESIRI matrices typically snake back and forth (serpentine layout)
      // We calculate the correct 1D array index from X/Y coordinates
      int pixelIndex;
      if (x % 2 == 0) { // Even columns go up
        pixelIndex = (x * MATRIX_HEIGHT) + y;
      } else {          // Odd columns go down
        pixelIndex = (x * MATRIX_HEIGHT) + (MATRIX_HEIGHT - 1 - y);
      }
      
      // Assign a basic color scale based on height
      leds[pixelIndex] = CHSV(x * (255 / MATRIX_WIDTH), 255, 255);
    }
  }

  FastLED.show();
  
  // Target roughly 60 FPS
  vTaskDelay(16 / portTICK_PERIOD_MS); 
}
