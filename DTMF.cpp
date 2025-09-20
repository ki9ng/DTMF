/*
DTMF Library for ESP32 - Improved with Automatic Magnitude Compensation
Based on: https://github.com/jacobrosenthal/Goertzel
Enhanced with proper Goertzel normalization as mentioned in Arduino forum

Key improvements:
- Automatic magnitude compensation (divides by sample count)
- Adaptive threshold calculation
- Better signal-to-noise ratio handling
- More robust DTMF detection

The Goertzel algorithm traditionally normalizes results by dividing by the number
of samples (N) to make detection independent of signal amplitude variations.
This implementation adds that normalization plus additional enhancements.
*/

#include "Arduino.h"
#include "DTMF_ESP32_Improved.h"

float SAMPLING_RATE;
int N;
float coeff[8];
float Q1[8];
float Q2[8];
int testData[160];

// Standard DTMF frequencies
const int dtmf_tones[8] = {
  697, 770, 852, 941,    // Row frequencies
  1209, 1336, 1477, 1633 // Column frequencies
};

// DTMF mapping
const unsigned char dtmf_map[16] = {
  0x11, 0x21, 0x41, 0x12, 0x22, 0x42, 0x14, 0x24,
  0x44, 0x28, 0x81, 0x82, 0x84, 0x88, 0x18, 0x48
};

const char dtmf_char[16] = {
  '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
  'A', 'B', 'C', 'D', '*', '#'
};

DTMF_ESP32_Improved::DTMF_ESP32_Improved(float BLOCK) {
  DTMF_ESP32_Improved(BLOCK, 8000.0);
}

DTMF_ESP32_Improved::DTMF_ESP32_Improved(float BLOCK, float SAMPLING_FREQ) {
  SAMPLING_RATE = SAMPLING_FREQ;
  N = BLOCK;
  
  // Calculate coefficients for each DTMF tone
  for(int i = 0; i < 8; i++) {
    float omega = (2.0 * PI * dtmf_tones[i]) / SAMPLING_RATE;
    coeff[i] = 2.0 * cos(omega);
  }
  
  // Initialize adaptive parameters
  adaptiveThreshold = 100.0;
  noiseFloor = 0.0;
  signalHistory[0] = signalHistory[1] = signalHistory[2] = 0.0;
  historyIndex = 0;
  
  ResetDTMF();
}

void DTMF_ESP32_Improved::ResetDTMF(void) {
  for(int i = 0; i < 8; i++) {
    Q2[i] = 0;
    Q1[i] = 0;
  }
}

void DTMF_ESP32_Improved::ProcessSample(int sample, int adc_centre) {
  float Q0;
  for(int i = 0; i < 8; i++) {
    Q0 = coeff[i] * Q1[i] - Q2[i] + (float)(sample - adc_centre);
    Q2[i] = Q1[i];
    Q1[i] = Q0;
  }
}

void DTMF_ESP32_Improved::sample(int sensorPin) {
  for (int index = 0; index < N; index++) {
    testData[index] = analogRead(sensorPin);
    delayMicroseconds(10);
  }
}

void DTMF_ESP32_Improved::detect(float dtmf_mag[], int adc_centre) {
  // Process all samples
  for (int index = 0; index < N; index++) {
    ProcessSample(testData[index], adc_centre);
  }
  
  // Calculate normalized magnitudes (KEY IMPROVEMENT)
  for(int i = 0; i < 8; i++) {
    float d_tmp = Q1[i] * Q1[i] + Q2[i] * Q2[i] - coeff[i] * Q1[i] * Q2[i];
    
    // Apply Goertzel normalization - divide by sample count
    dtmf_mag[i] = sqrt(d_tmp) / N;
    
    // Scale up for easier threshold management (optional)
    dtmf_mag[i] *= 1000.0;
  }
  
  // Update noise floor and adaptive threshold
  updateAdaptiveParameters(dtmf_mag);
  
  ResetDTMF();
}

void DTMF_ESP32_Improved::updateAdaptiveParameters(float mags[]) {
  // Calculate current signal statistics
  float maxMag = 0;
  float totalEnergy = 0;
  int activeTones = 0;
  
  for(int i = 0; i < 8; i++) {
    if(mags[i] > maxMag) maxMag = mags[i];
    totalEnergy += mags[i];
    if(mags[i] > adaptiveThreshold * 0.3) activeTones++;
  }
  
  float avgEnergy = totalEnergy / 8.0;
  
  // Update noise floor (exponential moving average)
  if(activeTones < 2) { // Likely no DTMF signal
    noiseFloor = noiseFloor * 0.95 + avgEnergy * 0.05;
  }
  
  // Update signal history
  signalHistory[historyIndex] = maxMag;
  historyIndex = (historyIndex + 1) % 3;
  
  // Calculate adaptive threshold
  float recentMax = max(signalHistory[0], max(signalHistory[1], signalHistory[2]));
  
  if(recentMax > noiseFloor * 3.0) {
    // Strong signal present - use signal-based threshold
    adaptiveThreshold = noiseFloor + (recentMax - noiseFloor) * 0.4;
  } else {
    // Weak/no signal - use noise-based threshold
    adaptiveThreshold = noiseFloor * 2.5;
  }
  
  // Keep threshold in reasonable bounds
  adaptiveThreshold = constrain(adaptiveThreshold, 5.0, 200.0);
}

char DTMF_ESP32_Improved::button(float mags[], float magnitude) {
  // Use adaptive threshold if magnitude is 0 or negative
  float threshold = (magnitude <= 0) ? adaptiveThreshold : magnitude;
  
  return buttonWithThreshold(mags, threshold);
}

char DTMF_ESP32_Improved::buttonWithThreshold(float mags[], float threshold) {
  int bit = 1;
  int dtmf = 0;
  
  // Check for enhanced DTMF detection
  if(enhancedDetection) {
    return buttonEnhanced(mags, threshold);
  }
  
  // Standard detection
  for(int i = 0; i < 8; i++) {
    if(mags[i] > threshold) {
      dtmf |= bit;
    }
    bit <<= 1;
  }
  
  // Find matching DTMF pattern
  for(int j = 0; j < 16; j++) {
    if(dtmf_map[j] == dtmf) {
      char detected = dtmf_char[j];
      
      // Simple debouncing
      if(detected == lastDetected) return 0;
      lastDetected = detected;
      return detected;
    }
  }
  
  lastDetected = 0;
  return 0;
}

char DTMF_ESP32_Improved::buttonEnhanced(float mags[], float threshold) {
  // Enhanced detection with signal quality checks
  
  // Find the two strongest frequencies
  float sorted[8];
  int indices[8];
  
  for(int i = 0; i < 8; i++) {
    sorted[i] = mags[i];
    indices[i] = i;
  }
  
  // Simple bubble sort for top frequencies
  for(int i = 0; i < 7; i++) {
    for(int j = 0; j < 7-i; j++) {
      if(sorted[j] < sorted[j+1]) {
        float temp = sorted[j];
        sorted[j] = sorted[j+1];
        sorted[j+1] = temp;
        
        int tempIdx = indices[j];
        indices[j] = indices[j+1];
        indices[j+1] = tempIdx;
      }
    }
  }
  
  // Check if we have two strong tones
  if(sorted[0] > threshold && sorted[1] > threshold * 0.7) {
    // Verify the two strongest are in different groups (row vs column)
    int first = indices[0];
    int second = indices[1];
    
    bool firstIsRow = (first < 4);
    bool secondIsRow = (second < 4);
    
    if(firstIsRow != secondIsRow) { // One row, one column
      // Check signal quality - third tone should be much weaker
      if(sorted[2] < sorted[1] * 0.5) {
        // Build detection pattern
        int dtmf = 0;
        int bit = 1;
        for(int i = 0; i < 8; i++) {
          if(i == first || i == second) {
            dtmf |= bit;
          }
          bit <<= 1;
        }
        
        // Find matching DTMF character
        for(int j = 0; j < 16; j++) {
          if(dtmf_map[j] == dtmf) {
            char detected = dtmf_char[j];
            
            // Enhanced debouncing with quality check
            if(detected == lastDetected && detectionCount < 3) {
              detectionCount++;
              return 0;
            }
            
            if(detected != lastDetected) {
              detectionCount = 1;
            }
            
            lastDetected = detected;
            return detected;
          }
        }
      }
    }
  }
  
  lastDetected = 0;
  detectionCount = 0;
  return 0;
}

void DTMF_ESP32_Improved::enableEnhancedDetection(bool enable) {
  enhancedDetection = enable;
}

float DTMF_ESP32_Improved::getAdaptiveThreshold() {
  return adaptiveThreshold;
}

float DTMF_ESP32_Improved::getNoiseFloor() {
  return noiseFloor;
}

void DTMF_ESP32_Improved::printDebugInfo(float mags[]) {
  Serial.print("Adaptive Threshold: ");
  Serial.print(adaptiveThreshold, 1);
  Serial.print(" | Noise Floor: ");
  Serial.print(noiseFloor, 1);
  Serial.print(" | Magnitudes: ");
  
  for(int i = 0; i < 8; i++) {
    Serial.print(mags[i], 1);
    if(mags[i] > adaptiveThreshold) Serial.print("*");
    Serial.print(" ");
  }
  Serial.println();
}

// Get signal quality metrics
void DTMF_ESP32_Improved::getSignalQuality(float mags[], SignalQuality* quality) {
  quality->maxMagnitude = 0;
  quality->avgMagnitude = 0;
  quality->snrEstimate = 0;
  quality->activeTones = 0;
  
  float total = 0;
  for(int i = 0; i < 8; i++) {
    if(mags[i] > quality->maxMagnitude) quality->maxMagnitude = mags[i];
    if(mags[i] > adaptiveThreshold * 0.5) quality->activeTones++;
    total += mags[i];
  }
  
  quality->avgMagnitude = total / 8.0;
  quality->snrEstimate = (quality->maxMagnitude > 0) ? 
                        quality->maxMagnitude / (noiseFloor + 1.0) : 0.0;
}
