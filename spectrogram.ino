// =============================================================================
//  ESP32 + MAX4466 → Real-Time Audio Spectrogram on WS2812B LED Matrix
//  Built with PlatformIO + Arduino framework
//
//  WIRING:
//    MAX4466 OUT  → GPIO35  (ADC1_CH7, input-only pin — analog read only)
//    MAX4466 VCC  → 3.3V
//    MAX4466 GND  → GND
//    LED Matrix DATA → GPIO18
//    LED Matrix VCC  → 5V (use external supply for large matrices!)
//    LED Matrix GND  → GND (share GND with ESP32)
//
//  REQUIRED LIBRARIES (auto-installed via platformio.ini lib_deps):
//    - FastLED        by Daniel Garcia   (LED control)
//    - arduinoFFT     by Enrique Gomez   (frequency analysis)
// =============================================================================

#include <Arduino.h>
#include <driver/i2s.h>   // ESP-IDF I2S driver, built into ESP32 Arduino core — no install needed
#include <FastLED.h>
#include <arduinoFFT.h>

// =============================================================================
//  MATRIX CONFIGURATION
//  Adjust these to match your physical hardware.
// =============================================================================

#define LED_PIN         18          // GPIO pin connected to matrix DATA line
#define NUM_COLS        32          // Number of columns (frequency bands displayed)
#define NUM_ROWS        8           // Number of rows (bar height resolution)
#define NUM_LEDS        (NUM_COLS * NUM_ROWS)
#define LED_TYPE        WS2812B     // Your LED chipset — common alternatives: WS2811, SK6812
#define COLOR_ORDER     GRB         // Pixel color byte order — try RGB or BGR if colors look wrong
#define MAX_BRIGHTNESS  150         // 0–255. Stay ≤150 unless using a dedicated 5V power supply.
                                    // At full brightness, 256 WS2812B LEDs can draw ~15A!

// =============================================================================
//  ADC / I2S (MICROPHONE) CONFIGURATION
// =============================================================================

#define I2S_PORT        I2S_NUM_0
#define ADC_CHANNEL     ADC1_CHANNEL_7  // Maps to GPIO35. If you move the mic wire,
                                         // update this — see ESP32 ADC1 channel map:
                                         // CH0=GPIO36, CH3=GPIO39, CH4=GPIO32,
                                         // CH5=GPIO33, CH6=GPIO34, CH7=GPIO35

#define SAMPLE_RATE     40000       // Samples per second (Hz).
                                    // ↑ Higher = captures higher frequencies (max ~20kHz for music)
                                    //   but uses more CPU per loop iteration.
                                    // ↓ Lower = faster FFT, but misses high-pitched sounds.
                                    // 40000 Hz captures up to 20kHz (full human hearing range).
                                    // Try 20000 if you only care about voice/bass.

#define FFT_SAMPLES     512         // Number of audio samples per FFT frame. MUST be a power of 2.
                                    // ↑ Higher (1024, 2048) = finer frequency resolution,
                                    //   smoother spectrum, but slower loop (more CPU).
                                    // ↓ Lower (256) = faster/snappier response, less freq detail.
                                    // 512 is a solid middle ground for music visualization.
                                    //
                                    // Frequency resolution = SAMPLE_RATE / FFT_SAMPLES
                                    // At 40000Hz / 512 = ~78Hz per bin. Each bin covers 78Hz.

#define DMA_BUF_LEN     256         // I2S DMA buffer length in samples. Must be power of 2, max 1024.
                                    // Smaller = lower latency but more CPU interrupts.
#define DMA_BUF_COUNT   4           // Number of DMA buffers. Total DMA RAM = BUF_LEN × BUF_COUNT × 2 bytes.

// =============================================================================
//  SPECTROGRAM VISUAL TUNING
//  These are the main knobs to tweak if the display looks wrong.
// =============================================================================

#define NOISE_FLOOR     300.0f      // FFT magnitudes below this value are treated as silence.
                                    // ↑ Raise this if LEDs flicker randomly in a quiet room.
                                    // ↓ Lower this if the display barely reacts to quiet sounds.
                                    // Typical range: 100–600 depending on your mic gain trim pot.

#define GAIN            2.0f        // Amplification applied after the noise floor is subtracted.
                                    // ↑ Raise if bars stay stubbornly short even with loud audio.
                                    // ↓ Lower if bars always slam to the top and stay there.

#define MAX_MAGNITUDE   4000.0f     // Expected maximum FFT magnitude (after gain).
                                    // This sets the "ceiling" — a bar reaches full height when
                                    // magnitude hits this value. Tune alongside GAIN:
                                    // ↓ Lower = bars reach top more easily (more sensitive)
                                    // ↑ Higher = bars only max out at very loud sounds

#define DECAY           0.78f       // Controls how fast bars fall after a peak (0.0–1.0).
                                    // ↑ Higher (e.g. 0.9) = bars hang longer, trails persist
                                    // ↓ Lower  (e.g. 0.5) = bars drop instantly, very snappy
                                    // Good music viz range: 0.7–0.85

// =============================================================================
//  GLOBALS
// =============================================================================

CRGB leds[NUM_LEDS];

// FFT input/output arrays. vReal gets audio samples, vImag starts at 0.
// After FFT.compute(), vReal contains the frequency magnitudes.
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

// ArduinoFFT v2.x uses a constructor-based API (not static methods).
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE);

// Smoothed bar heights per column, range 0.0–1.0
float barHeights[NUM_COLS] = {0};

// =============================================================================
//  I2S INITIALIZATION
//  Configures the ESP32's built-in I2S peripheral to drive the ADC continuously.
//  This gives us much better sample timing than analogRead() in a loop,
//  which has jitter and can't reliably hit 40kHz.
// =============================================================================
void setupI2S() {
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono mic — only use left channel
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = DMA_BUF_COUNT,
    .dma_buf_len          = DMA_BUF_LEN,
    .use_apll             = false,                       // APLL gives cleaner clock but isn't needed here
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_adc_enable(I2S_PORT);
}

// =============================================================================
//  AUDIO CAPTURE
//  Fills vReal[] with FFT_SAMPLES audio samples from the ADC via I2S DMA.
//  Each raw 16-bit I2S word holds a 12-bit ADC value in the lower 12 bits.
//  We convert to signed (-2048 to +2047) so FFT works correctly.
// =============================================================================
void captureAudio() {
  uint16_t rawBuf[DMA_BUF_LEN];
  int samplesRead = 0;

  while (samplesRead < FFT_SAMPLES) {
    size_t bytesRead = 0;
    // portMAX_DELAY = wait indefinitely until DMA buffer is ready (non-blocking spin)
    i2s_read(I2S_PORT, rawBuf, sizeof(rawBuf), &bytesRead, portMAX_DELAY);
    int count = bytesRead / sizeof(uint16_t);

    for (int i = 0; i < count && samplesRead < FFT_SAMPLES; i++) {
      // Mask lower 12 bits → unsigned 0–4095
      // Subtract 2048 → signed -2048 to +2047 (centers waveform at zero for FFT)
      vReal[samplesRead] = (double)((rawBuf[i] & 0x0FFF) - 2048);
      vImag[samplesRead] = 0.0;  // Imaginary part must be zeroed before each FFT run
      samplesRead++;
    }
  }
}

// =============================================================================
//  LED INDEX MAPPING
//
//  Translates logical (col, row) coordinates to a physical LED index.
//  col = 0 is leftmost (bass), col = NUM_COLS-1 is rightmost (treble)
//  row = 0 is the bottom bar, row = NUM_ROWS-1 is the top
//
//  This assumes SERPENTINE wiring (the most common matrix layout):
//    Row 0 (bottom): LEDs 0  → 31   left to right
//    Row 1:          LEDs 63 → 32   right to left  ← direction flips each row
//    Row 2:          LEDs 64 → 95   left to right
//    ...etc
//
//  ⚠ IF YOUR MATRIX LOOKS SCRAMBLED: Your wiring layout differs. Common fixes:
//    - Non-serpentine (all rows left→right): remove the `if/else`, always use
//      `return displayRow * NUM_COLS + col;`
//    - Origin at top-left: remove the `displayRow` flip, use `row` directly
//    - Vertical wiring (columns run top→bottom): swap row/col math
// =============================================================================
int getLEDIndex(int col, int row) {
  // Flip vertically: row 0 = bottom of matrix = last physical row
  int displayRow = (NUM_ROWS - 1) - row;

  if (displayRow % 2 == 0) {
    return displayRow * NUM_COLS + col;               // Even rows: left → right
  } else {
    return displayRow * NUM_COLS + (NUM_COLS - 1 - col); // Odd rows: right → left
  }
}

// =============================================================================
//  DRAW SPECTROGRAM BARS
//
//  Renders one vertical bar per column using barHeights[] (0.0–1.0).
//  Color goes from blue (quiet/low rows) up through green/yellow to red (loud/top).
//  This mimics a classic spectrum analyzer look.
// =============================================================================
void drawBars() {
  FastLED.clear();  // Wipe all LEDs to black before redrawing

  for (int col = 0; col < NUM_COLS; col++) {
    // Convert normalized height (0.0–1.0) to a pixel row count
    int barTop = constrain((int)(barHeights[col] * (NUM_ROWS - 1) + 0.5f), 0, NUM_ROWS - 1);

    for (int row = 0; row <= barTop; row++) {
      // Map row position to hue: 160 (blue) at bottom → 0 (red) at top
      // HSV color wheel: 0=red, 32=orange, 64=yellow, 96=green, 160=blue
      uint8_t hue = map(row, 0, NUM_ROWS - 1, 160, 0);
      leds[getLEDIndex(col, row)] = CHSV(hue, 255, 255);

      // 💡 Alternative color schemes to try:
      //   All green:  CRGB::Green
      //   Heat map:   CHSV(map(row, 0, NUM_ROWS-1, 96, 0), 255, 255)  // green→red only
      //   Cool blue:  CHSV(160, 255, map(row, 0, NUM_ROWS-1, 80, 255)) // dim→bright blue
    }
  }

  FastLED.show();  // Push buffer to LEDs over the data wire
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Spectrogram init...");

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  setupI2S();

  Serial.println("Ready — listening for audio.");
}

// =============================================================================
//  MAIN LOOP  (runs continuously, ~30–60 times per second at these settings)
// =============================================================================
void loop() {

  // ── Step 1: Fill vReal[] with fresh audio samples via I2S DMA ──────────────
  captureAudio();

  // ── Step 2: Apply Hann window ───────────────────────────────────────────────
  // A window function tapers the sample block to zero at both ends.
  // Without this, the FFT "sees" a hard edge at the buffer boundary and
  // produces false high-frequency noise (spectral leakage). Always use this.
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);

  // ── Step 3: Run the FFT ─────────────────────────────────────────────────────
  // Transforms FFT_SAMPLES time-domain samples → FFT_SAMPLES/2 frequency bins.
  // After this call, vReal[i] holds the magnitude of frequency bin i.
  // Bin i corresponds to frequency: i × (SAMPLE_RATE / FFT_SAMPLES) Hz
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();  // Converts complex (real+imag) output to scalar magnitudes

  // ── Step 4: Map frequency bins → display columns ────────────────────────────
  // We use a LOGARITHMIC scale so low frequencies (bass) get more columns.
  // This matches human pitch perception — an octave always looks the same width.
  // A linear scale would cram all the bass into the first 1–2 columns.
  //
  // halfSamples = the number of useful FFT output bins (Nyquist limit)
  int halfSamples = FFT_SAMPLES / 2;  // = 256 bins, each covering ~78Hz at 40kHz/512

  for (int col = 0; col < NUM_COLS; col++) {

    // Log-scale bin range for this column:
    // Column 0 maps to the very low bins (bass), column NUM_COLS-1 maps to high bins (treble)
    float freqStart = powf((float)halfSamples, (float)col       / NUM_COLS);
    float freqEnd   = powf((float)halfSamples, (float)(col + 1) / NUM_COLS);

    int binStart = max(1, (int)freqStart);           // Skip bin 0 — it's the DC offset (useless)
    int binEnd   = max(binStart + 1, (int)freqEnd);  // Ensure at least 1 bin per column
    binEnd       = min(binEnd, halfSamples);

    // Average magnitudes across all bins in this column's frequency range.
    // Averaging (rather than taking the peak) gives a smoother, less spiky display.
    float sum = 0;
    for (int b = binStart; b < binEnd; b++) {
      sum += (float)vReal[b];
    }
    float avg = sum / (float)(binEnd - binStart);

    // Apply noise floor: ignore everything below ambient electrical noise level
    // Apply gain: amplify the remaining signal
    float level = max(0.0f, avg - NOISE_FLOOR) * GAIN;

    // Normalize to 0.0–1.0 using MAX_MAGNITUDE as the ceiling
    float normalized = constrain(level / MAX_MAGNITUDE, 0.0f, 1.0f);

    // Smooth the bar height with exponential decay:
    //   - Bars RISE instantly to the new level (snappy attack)
    //   - Bars FALL gradually using the DECAY multiplier (smooth release)
    if (normalized > barHeights[col]) {
      barHeights[col] = normalized;
    } else {
      barHeights[col] = barHeights[col] * DECAY + normalized * (1.0f - DECAY);
    }
  }

  // ── Step 5: Render bars to the LED matrix ───────────────────────────────────
  drawBars();

  // Optional: print the loudest frequency to Serial Monitor for debugging
   double peakHz = FFT.majorPeak();
   Serial.printf("Peak: %.1f Hz\n", peakHz);
}
