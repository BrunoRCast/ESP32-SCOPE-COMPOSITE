/*
ESP32 Oscilloscope Simulator - Tektronix TDS420 Style
Enhanced version with improved display and functionality

Hardware Connections:
- GPIO25: Composite video output (connect to video signal line)
- GPIO36: Channel 1 analog input (ADC1_CH0) - Yellow probe
- GPIO39: Channel 2 analog input (ADC1_CH3) - Blue probe  
- GPIO34: Channel 3 analog input (ADC1_CH6) - Pink probe
- GPIO35: Channel 4 analog input (ADC1_CH7) - Green probe
*/

#include <ESP_8_BIT_GFX.h>
#include <driver/adc.h>

// Create an instance of the graphics library
ESP_8_BIT_GFX videoOut(false /* = PAL M */, 8 /* = RGB332 color */);

// Trigger modes
enum TriggerMode {
  TRIG_AUTO,
  TRIG_NORMAL,
  TRIG_SINGLE
};

enum TriggerSlope {
  TRIG_RISING,
  TRIG_FALLING
};

enum CouplingType {
  COUPLING_DC,
  COUPLING_AC
};

// Enhanced Oscilloscope parameters
struct OscilloscopeConfig {
  bool channelEnabled[4] = {true, false, false, false};
  uint16_t vScale[4] = {100, 100, 100, 100}; // mV/div
  float position[4] = {0, 0, 0, 0}; // Vertical position in divisions
  CouplingType coupling[4] = {COUPLING_DC, COUPLING_DC, COUPLING_DC, COUPLING_DC};
  float hScale = 0.5; // µs/div (supports decimal values)
  uint8_t triggerChannel = 1;
  uint16_t triggerLevel = 30; // mV
  TriggerMode triggerMode = TRIG_AUTO;
  TriggerSlope triggerSlope = TRIG_RISING;
  bool running = true;
  bool autosetRequested = false;
  uint32_t autoTriggerTimeout = 100000; // Auto trigger timeout in µs
  uint32_t lastTriggerTime = 0;
} config;

// Sample buffer for each channel
#define BUFFER_SIZE 512
#define DISPLAY_SAMPLES 256
uint16_t sampleBuffer[4][BUFFER_SIZE];
uint16_t displayBuffer[4][DISPLAY_SAMPLES];
uint16_t bufferIndex = 0;
uint16_t triggerIndex = 0;
bool bufferReady = false;
bool triggered = false;
uint32_t lastSampleTime = 0;

// AC coupling filters (simple high-pass)
float dcFilter[4] = {2048, 2048, 2048, 2048};
const float filterAlpha = 0.001; // Low-pass filter for DC component

// Colors for each channel (RGB332) - TDS420 style
uint8_t channelColors[4] = {
  0xFC, // Yellow (CH1) - bright yellow
  0x1F, // Cyan (CH2) - bright cyan  
  0xE3, // Magenta (CH3) - bright magenta
  0x1C  // Green (CH4) - bright green
};

// ADC channels mapping
adc1_channel_t adcChannels[4] = {
  ADC1_CHANNEL_0, // GPIO36
  ADC1_CHANNEL_3, // GPIO39
  ADC1_CHANNEL_6, // GPIO34  
  ADC1_CHANNEL_7  // GPIO35
};

// Task handles
TaskHandle_t acquisitionTaskHandle;

// Voltage scale values in mV/div
const uint16_t vScaleValues[] = {5, 10, 20, 50, 100, 200, 500, 1000, 2000};
const uint8_t vScaleCount = sizeof(vScaleValues) / sizeof(vScaleValues[0]);

// Time scale values in µs/div (includes decimal values)  
const float hScaleValues[] = {0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000};
const uint8_t hScaleCount = sizeof(hScaleValues) / sizeof(hScaleValues[0]);

// Adjusted display constants for NTSC safe zone
const int SCREEN_WIDTH = 250;   // Reduced width for NTSC
const int SCREEN_HEIGHT = 240;  // Standard height
const int GRID_START_X = 20;    // Left margin
const int GRID_START_Y = 30;    // Top margin
const int GRID_WIDTH = 192;     // 8 divisions * 24 pixels
const int GRID_HEIGHT = 180;    // 6 divisions * 30 pixels
const int GRID_DIV_SIZE = 24;   // Adjusted division size

void setup() {
  Serial.begin(115200);
  Serial.println("\n" + String((char)201) + String((char)205) + " QUERO CAFE " + String((char)205) + String((char)187));
  Serial.println(String((char)186) + "  4-Channel Digital Storage Scope UOWWW    " + String((char)186));
  Serial.println(String((char)200) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)205) + String((char)188));
  
  // Initial setup of graphics library
  videoOut.begin();
  videoOut.fillScreen(0x00);
  
  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  for(int i = 0; i < 4; i++) {
    adc1_config_channel_atten(adcChannels[i], ADC_ATTEN_DB_11);
  }
  
  // Create acquisition task on core 0
  xTaskCreatePinnedToCore(
    acquisitionTask,
    "Acquisition",
    8192,
    NULL,
    3, // Higher priority
    &acquisitionTaskHandle,
    0  // Core 0
  );
  
  Serial.println("System initialized. Type HELP for commands.");
  printStatus();
}

void loop() {
  // Video generation on Core 1
  videoOut.waitForFrame();
  
  // Process serial commands
  processSerialCommands();
  
  // Handle autoset
  if(config.autosetRequested) {
    performAutoset();
    config.autosetRequested = false;
  }
  
  // Draw oscilloscope display
  drawOscilloscopeDisplay();
}

void acquisitionTask(void *parameter) {
  uint32_t sampleInterval;
  uint32_t autoTriggerStart = micros();
  bool waitingForTrigger = false;
  
  while(true) {
    if(!config.running && config.triggerMode != TRIG_SINGLE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    
    // Calculate sample interval based on horizontal scale
    sampleInterval = (uint32_t)(config.hScale * 1000.0 / (DISPLAY_SAMPLES / 8.0)); // 8 divisions
    if(sampleInterval < 5) sampleInterval = 5; // Minimum 5µs
    
    uint32_t currentTime = micros();
    if(currentTime - lastSampleTime >= sampleInterval) {
      
      // Read all channels
      for(int ch = 0; ch < 4; ch++) {
        uint16_t raw = adc1_get_raw(adcChannels[ch]);
        
        // Apply AC coupling if enabled
        if(config.coupling[ch] == COUPLING_AC) {
          // Update DC component with low-pass filter
          dcFilter[ch] = dcFilter[ch] * (1.0 - filterAlpha) + raw * filterAlpha;
          // Remove DC component
          raw = (uint16_t)constrain(raw - dcFilter[ch] + 2048, 0, 4095);
        }
        
        sampleBuffer[ch][bufferIndex] = raw;
      }
      
      // Trigger logic
      bool triggerEvent = false;
      
      if(config.triggerMode != TRIG_AUTO || !waitingForTrigger) {
        if(bufferIndex > 2 && config.channelEnabled[config.triggerChannel - 1]) {
          uint16_t triggerSample = sampleBuffer[config.triggerChannel - 1][bufferIndex];
          uint16_t prevSample = sampleBuffer[config.triggerChannel - 1][bufferIndex - 1];
          
          // Convert trigger level from mV to ADC units
          uint16_t triggerADC = (config.triggerLevel * 4095) / 3300;
          
          if(config.triggerSlope == TRIG_RISING) {
            triggerEvent = (prevSample < triggerADC && triggerSample >= triggerADC);
          } else {
            triggerEvent = (prevSample > triggerADC && triggerSample <= triggerADC);
          }
          
          if(triggerEvent) {
            triggered = true;
            triggerIndex = bufferIndex;
            waitingForTrigger = false;
            autoTriggerStart = currentTime;
          }
        }
      }
      
      // Auto trigger timeout
      if(config.triggerMode == TRIG_AUTO && !triggered && 
         (currentTime - autoTriggerStart) > config.autoTriggerTimeout) {
        triggered = true;
        triggerIndex = bufferIndex;
        autoTriggerStart = currentTime;
      }
      
      bufferIndex++;
      if(bufferIndex >= BUFFER_SIZE) {
        bufferIndex = 0;
      }
      
      // Check if we have enough post-trigger samples
      if(triggered) {
        uint16_t postTriggerSamples = (bufferIndex >= triggerIndex) ? 
                                     (bufferIndex - triggerIndex) : 
                                     (BUFFER_SIZE - triggerIndex + bufferIndex);
        
        if(postTriggerSamples >= DISPLAY_SAMPLES / 2) {
          // Copy data to display buffer
          copyToDisplayBuffer();
          bufferReady = true;
          
          if(config.triggerMode == TRIG_SINGLE) {
            config.running = false;
          }
          
          triggered = false;
          waitingForTrigger = true;
        }
      }
      
      lastSampleTime = currentTime;
    }
    
    vTaskDelay(1);
  }
}

void copyToDisplayBuffer() {
  // Copy samples around trigger point to display buffer
  int startIndex = (triggerIndex >= DISPLAY_SAMPLES / 4) ? 
                   (triggerIndex - DISPLAY_SAMPLES / 4) : 
                   (BUFFER_SIZE + triggerIndex - DISPLAY_SAMPLES / 4);
  
  for(int ch = 0; ch < 4; ch++) {
    for(int i = 0; i < DISPLAY_SAMPLES; i++) {
      int sourceIndex = (startIndex + i) % BUFFER_SIZE;
      displayBuffer[ch][i] = sampleBuffer[ch][sourceIndex];
    }
  }
}

void drawOscilloscopeDisplay() {
  // Clear screen with black background
  videoOut.fillScreen(0x00);
  
  // Draw title bar
  drawTitleBar();
  
  // Draw graticule (grid)
  drawGraticule();
  
  // Draw channel information
  drawChannelInfo();
  
  // Draw waveforms
  drawWaveforms();
  
  // Draw status information
  drawStatusInfo();
  
  // Draw trigger indicator
  drawTriggerInfo();
}

void drawTitleBar() {
  // Draw title bar similar to TDS420
  videoOut.fillRect(0, 12, SCREEN_WIDTH, 16, 0x92); // Gray background
  videoOut.setTextColor(0x00); // Black text
  videoOut.setTextSize(1);
  videoOut.setCursor(2, 16);
  videoOut.print("BRC-SCOPE");
  
  // Show acquisition status
  videoOut.setCursor(150, 16);
  if(config.running) {
    videoOut.print("RUN");
  } else {
    videoOut.print("STOP");
  }
  
  // Show trigger status
  videoOut.setCursor(180, 16);
  if(triggered) {
    videoOut.print("TRIG");
  } else if(config.triggerMode == TRIG_AUTO) {
    videoOut.print("AUTO");
  } else {
    videoOut.print("READY");
  }
}

void drawGraticule() {
  // Draw graticule grid (8x6 divisions) - TDS420 style
  uint8_t gridColor = 0x49; // Dark gray
  uint8_t centerColor = 0x6D; // Brighter gray for center lines
  uint8_t tickColor = 0x24; // Very dark gray
  
  // Main grid lines
  // Vertical grid lines
  for(int i = 0; i <= 8; i++) {
    int x = GRID_START_X + i * GRID_DIV_SIZE;
    uint8_t color = (i == 4) ? centerColor : gridColor;
    videoOut.drawLine(x, GRID_START_Y, x, GRID_START_Y + GRID_HEIGHT, color);
  }
  
  // Horizontal grid lines
  for(int i = 0; i <= 6; i++) {
    int y = GRID_START_Y + i * (GRID_HEIGHT/6);
    uint8_t color = (i == 3) ? centerColor : gridColor;
    videoOut.drawLine(GRID_START_X, y, GRID_START_X + GRID_WIDTH, y, color);
  }
  
  // Minor tick marks (5 per division)
  for(int i = 0; i < 8; i++) {
    for(int j = 1; j < 5; j++) {
      int x = GRID_START_X + i * GRID_DIV_SIZE + j * (GRID_DIV_SIZE / 5);
      int centerY = GRID_START_Y + 3 * (GRID_HEIGHT/6);
      videoOut.drawLine(x, centerY - 2, x, centerY + 2, tickColor);
    }
  }
  
  for(int i = 0; i < 6; i++) {
    for(int j = 1; j < 5; j++) {
      int y = GRID_START_Y + i * (GRID_HEIGHT/6) + j * ((GRID_HEIGHT/6) / 5);
      int centerX = GRID_START_X + 4 * GRID_DIV_SIZE;
      videoOut.drawLine(centerX - 2, y, centerX + 2, y, tickColor);
    }
  }
}

void drawChannelInfo() {
  // Draw channel information on the left side - TDS420 style
  videoOut.setTextSize(1);
  videoOut.setTextWrap(false);
  
  const char* chLabels[] = {"1", "2", "3", "4"};
  const char* couplingLabels[] = {"DC", "AC"};
  
  for(int ch = 0; ch < 4; ch++) {
    int yPos = 30 + ch * 30;
    
    if(config.channelEnabled[ch]) {
      videoOut.setTextColor(channelColors[ch]);
      
      // Channel number with colored background
      videoOut.fillRect(2, yPos - 2, 12, 10, channelColors[ch]);
      videoOut.setTextColor(0x00); // Black text on colored background
      videoOut.setCursor(4, yPos);
      videoOut.print(chLabels[ch]);
      
      // Reset to channel color for other info
      videoOut.setTextColor(channelColors[ch]);
      
      // Voltage scale
      videoOut.setCursor(2, yPos + 10);
      if(config.vScale[ch] >= 1000) {
        videoOut.print(config.vScale[ch] / 1000);
        videoOut.print("V");
      } else {
        videoOut.print(config.vScale[ch]);
        videoOut.print("m");
      }
      
      // Coupling
      videoOut.setCursor(2, yPos + 18);
      videoOut.print(couplingLabels[config.coupling[ch]]);
      
      // Position indicator
      if(config.position[ch] != 0) {
        videoOut.setCursor(2, yPos + 26);
        if(config.position[ch] > 0) {
          videoOut.print("+");
        }
        videoOut.print(config.position[ch], 1);
      }
      
    } else {
      // Show disabled channel in dark color
      videoOut.setTextColor(0x49);
      videoOut.setCursor(4, yPos);
      videoOut.print(chLabels[ch]);
      videoOut.setCursor(2, yPos + 10);
      videoOut.print("OFF");
    }
  }
}

void drawWaveforms() {
  if(!bufferReady) return;
  
  const int centerY = GRID_START_Y + GRID_HEIGHT / 2;
  
  for(int ch = 0; ch < 4; ch++) {
    if(!config.channelEnabled[ch]) continue;
    
    uint8_t color = channelColors[ch];
    
    // Draw waveform with anti-aliasing effect
    for(int i = 1; i < DISPLAY_SAMPLES - 1; i++) {
      // Convert sample index to screen X coordinate
      int x1 = GRID_START_X + ((i - 1) * GRID_WIDTH) / (DISPLAY_SAMPLES - 1);
      int x2 = GRID_START_X + (i * GRID_WIDTH) / (DISPLAY_SAMPLES - 1);
      
      // Convert ADC values to screen Y coordinates
      int y1 = adcToScreenY(displayBuffer[ch][i - 1], ch, centerY);
      int y2 = adcToScreenY(displayBuffer[ch][i], ch, centerY);
      
      // Clamp to display area
      y1 = constrain(y1, GRID_START_Y, GRID_START_Y + GRID_HEIGHT);
      y2 = constrain(y2, GRID_START_Y, GRID_START_Y + GRID_HEIGHT);
      
      // Draw line segment
      videoOut.drawLine(x1, y1, x2, y2, color);
      
      // Add intensity for thick line effect
      if(abs(y2 - y1) > 1) {
        videoOut.drawLine(x1, y1 + 1, x2, y2 + 1, color & 0x77); // Dimmer
      }
    }
  }
  
  // Draw trigger position marker
  if(config.triggerMode != TRIG_AUTO) {
    int triggerX = GRID_START_X + GRID_WIDTH / 4; // 25% from left
    uint8_t trigColor = channelColors[config.triggerChannel - 1];
    
    // Draw trigger arrow
    videoOut.fillTriangle(triggerX - 3, GRID_START_Y - 8, 
                         triggerX + 3, GRID_START_Y - 8, 
                         triggerX, GRID_START_Y - 2, trigColor);
  }
}

int adcToScreenY(uint16_t adcValue, int channel, int centerY) {
  // Convert 12-bit ADC value (0-4095) to screen Y coordinate
  // ADC range: 0V = 0, 3.3V = 4095
  
  // Normalize ADC value to voltage (0V to 3.3V)
  float voltage = (adcValue * 3.3) / 4095.0;
  
  // Get voltage scale for this channel (convert mV to V)
  float vScale = config.vScale[channel] / 1000.0;
  
  // Calculate pixels per volt
  float pixelsPerVolt = (GRID_HEIGHT/6) / vScale;
  
  // Center the waveform around 1.65V (middle of 3.3V range)
  float centerVoltage = 1.65;
  
  // Apply vertical position offset
  float positionOffset = config.position[channel] * (GRID_HEIGHT/6);
  
  // Calculate Y position (invert because screen Y increases downward)
  int y = centerY - (int)((voltage - centerVoltage) * pixelsPerVolt) + positionOffset;
  
  return y;
}

void drawStatusInfo() {
  // Draw status information at the bottom - TDS420 style
  videoOut.setTextColor(0xFF); // White
  videoOut.setTextSize(1);
  
  // Time scale
  videoOut.setCursor(GRID_START_X, 220);
  videoOut.print("Time: ");
  if(config.hScale >= 1000) {
    videoOut.print(config.hScale / 1000, 1);
    videoOut.print("ms/div");
  } else if(config.hScale >= 1) {
    videoOut.print((int)config.hScale);
    videoOut.print("us/div");
  } else {
    videoOut.print(config.hScale, 1);
    videoOut.print("us/div");
  }
  
  // Sample rate
  videoOut.setCursor(GRID_START_X + 90, 220);
  float sampleRate = 1000000.0 / ((config.hScale * 1000.0) / (DISPLAY_SAMPLES / 8.0));
  if(sampleRate >= 1000000) {
    videoOut.print(sampleRate / 1000000, 1);
    videoOut.print("MS/s");
  } else if(sampleRate >= 1000) {
    videoOut.print(sampleRate / 1000, 1);
    videoOut.print("kS/s");
  } else {
    videoOut.print((int)sampleRate);
    videoOut.print("S/s");
  }
}

void drawTriggerInfo() {
  // Draw trigger information on the right side
  videoOut.setTextColor(0xFF); // White
  videoOut.setTextSize(1);
  
  int startX = GRID_START_X + GRID_WIDTH + 2;
  
  // Trigger channel
  videoOut.setCursor(startX, 30);
  videoOut.print("T:CH");
  videoOut.print(config.triggerChannel);
  
  // Trigger level
  videoOut.setCursor(startX, 40);
  videoOut.print("L:");
  videoOut.print(config.triggerLevel);
  videoOut.print("mV");
  
  // Trigger mode
  videoOut.setCursor(startX, 50);
  switch(config.triggerMode) {
    case TRIG_AUTO: videoOut.print("A"); break;
    case TRIG_NORMAL: videoOut.print("N"); break;
    case TRIG_SINGLE: videoOut.print("S"); break;
  }
  
  // Trigger slope
  videoOut.setCursor(startX, 60);
  videoOut.print(config.triggerSlope == TRIG_RISING ? "↑" : "↓");
}

void processSerialCommands() {
  if(!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  // Channel control
  if(command.startsWith("CH") && command.length() >= 5) {
    int ch = command.charAt(2) - '1';
    if(ch >= 0 && ch < 4) {
      if(command.endsWith("ON")) {
        config.channelEnabled[ch] = true;
        Serial.println("Channel " + String(ch+1) + " enabled");
      } else if(command.endsWith("OFF")) {
        config.channelEnabled[ch] = false;
        Serial.println("Channel " + String(ch+1) + " disabled");
      }
    }
  }
  // Voltage scale
  else if(command.startsWith("VSCALE")) {
    parseVScale(command);
  }
  // Horizontal scale
  else if(command.startsWith("HSCALE")) {
    parseHScale(command);
  }
  // Trigger channel
  else if(command.startsWith("TRIGGER")) {
    int spaceIndex = command.indexOf(' ');
    if(spaceIndex > 0) {
      int ch = command.substring(spaceIndex + 1).toInt();
      if(ch >= 1 && ch <= 4) {
        config.triggerChannel = ch;
        Serial.println("Trigger channel: " + String(ch));
      }
    }
  }
  // Trigger level
  else if(command.startsWith("TLEVEL")) {
    int spaceIndex = command.indexOf(' ');
    if(spaceIndex > 0) {
      int level = command.substring(spaceIndex + 1).toInt();
      if(level >= 0 && level <= 3300) {
        config.triggerLevel = level;
        Serial.println("Trigger level: " + String(level) + "mV");
      }
    }
  }
  // Coupling
  else if(command.startsWith("COUPLING")) {
    parseCoupling(command);
  }
  // Position
  else if(command.startsWith("POSITION")) {
    parsePosition(command);
  }
  // Trigger mode
  else if(command.startsWith("TMODE")) {
    parseTriggerMode(command);
  }
  // Trigger slope
  else if(command.startsWith("TSLOPE")) {
    parseTriggerSlope(command);
  }
  // Run/Stop
  else if(command == "RUN") {
    config.running = true;
    Serial.println("Acquisition started");
  }
  else if(command == "STOP") {
    config.running = false;
    Serial.println("Acquisition stopped");
  }
  // Autoset
  else if(command == "AUTOSET") {
    config.autosetRequested = true;
    Serial.println("Autoset requested");
  }
  // Help
  else if(command == "HELP") {
    printHelp();
  }
  // Status
  else if(command == "STATUS") {
    printStatus();
  }
  else {
    Serial.println("Unknown command. Type HELP for available commands.");
  }
}

void parseVScale(String command) {
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if(firstSpace > 0 && secondSpace > 0) {
    int ch = command.substring(firstSpace + 1, secondSpace).toInt() - 1;
    int scale = command.substring(secondSpace + 1).toInt();
    
    if(ch >= 0 && ch < 4) {
      bool validScale = false;
      for(int i = 0; i < vScaleCount; i++) {
        if(vScaleValues[i] == scale) {
          validScale = true;
          break;
        }
      }
      
      if(validScale) {
        config.vScale[ch] = scale;
        Serial.println("CH" + String(ch+1) + " voltage scale: " + String(scale) + "mV/div");
      } else {
        Serial.println("Invalid voltage scale. Valid: 5,10,20,50,100,200,500,1000,2000");
      }
    }
  }
}

void parseHScale(String command) {
  int spaceIndex = command.indexOf(' ');
  if(spaceIndex > 0) {
    float scale = command.substring(spaceIndex + 1).toFloat();
    
    bool validScale = false;
    for(int i = 0; i < hScaleCount; i++) {
      if(abs(hScaleValues[i] - scale) < 0.01) {
        validScale = true;
        break;
      }
    }
    
    if(validScale) {
      config.hScale = scale;
      Serial.println("Time scale: " + String(scale, 1) + "µs/div");
    } else {
      Serial.println("Invalid time scale. Valid: 0.5,1,2,5,10,20,50,100,200,500,1000,2000,5000");
    }
  }
}

void parseCoupling(String command) {
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if(firstSpace > 0 && secondSpace > 0) {
    int ch = command.substring(firstSpace + 1, secondSpace).toInt() - 1;
    String coupling = command.substring(secondSpace + 1);
    
    if(ch >= 0 && ch < 4) {
      if(coupling == "AC") {
        config.coupling[ch] = COUPLING_AC;
        dcFilter[ch] = 2048; // Reset DC filter
        Serial.println("CH" + String(ch+1) + " coupling: AC");
      } else if(coupling == "DC") {
        config.coupling[ch] = COUPLING_DC;
        Serial.println("CH" + String(ch+1) + " coupling: DC");
      } else {
        Serial.println("Invalid coupling type. Use AC or DC");
      }
    }
  }
}

void parsePosition(String command) {
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if(firstSpace > 0 && secondSpace > 0) {
    int ch = command.substring(firstSpace + 1, secondSpace).toInt() - 1;
    float position = command.substring(secondSpace + 1).toFloat();
    
    if(ch >= 0 && ch < 4 && position >= -3.0 && position <= 3.0) {
      config.position[ch] = position;
      Serial.println("CH" + String(ch+1) + " position: " + String(position, 1) + " div");
    } else {
      Serial.println("Invalid position. Must be between -3.0 and +3.0 divisions");
    }
  }
}

void parseTriggerMode(String command) {
  int spaceIndex = command.indexOf(' ');
  if(spaceIndex > 0) {
    String mode = command.substring(spaceIndex + 1);
    
    if(mode == "AUTO") {
      config.triggerMode = TRIG_AUTO;
      Serial.println("Trigger mode: AUTO");
    } else if(mode == "NORM") {
      config.triggerMode = TRIG_NORMAL;
      Serial.println("Trigger mode: NORMAL");
    } else if(mode == "SINGLE") {
      config.triggerMode = TRIG_SINGLE;
      Serial.println("Trigger mode: SINGLE");
    } else {
      Serial.println("Invalid trigger mode. Use AUTO, NORM, or SINGLE");
    }
  }
}

void parseTriggerSlope(String command) {
  int spaceIndex = command.indexOf(' ');
  if(spaceIndex > 0) {
    String slope = command.substring(spaceIndex + 1);
    
    if(slope == "RISE") {
      config.triggerSlope = TRIG_RISING;
      Serial.println("Trigger slope: RISING");
    } else if(slope == "FALL") {
      config.triggerSlope = TRIG_FALLING;
      Serial.println("Trigger slope: FALLING");
    } else {
      Serial.println("Invalid trigger slope. Use RISE or FALL");
    }
  }
}

void performAutoset() {
  // Simple autoset implementation
  Serial.println("Performing autoset...");
  
  // Enable first two channels, disable others
  config.channelEnabled[0] = true;
  config.channelEnabled[1] = false;
  config.channelEnabled[2] = false;
  config.channelEnabled[3] = false;
  
  // Set default voltage scales
  config.vScale[0] = 100;
  config.vScale[1] = 100;
  config.vScale[2] = 100;
  config.vScale[3] = 100;
  
  // Set default time scale
  config.hScale = 50;
  
  // Reset positions
  config.position[0] = 0;
  config.position[1] = 0;
  config.position[2] = 0;
  config.position[3] = 0;
  
  // Set default trigger
  config.triggerChannel = 1;
  config.triggerLevel = 1650;
  config.triggerMode = TRIG_AUTO;
  config.triggerSlope = TRIG_RISING;
  
  // Start acquisition
  config.running = true;
  
  Serial.println("Autoset complete");
  printStatus();
}

void printHelp() {
  Serial.println("\nAvailable commands:");
  Serial.println("CH<1-4> ON/OFF - Enable/disable channel");
  Serial.println("VSCALE <ch> <value> - Set voltage scale (mV/div)");
  Serial.println("HSCALE <value> - Set time scale (µs/div)");
  Serial.println("TRIGGER <ch> - Set trigger channel (1-4)");
  Serial.println("TLEVEL <value> - Set trigger level in mV (0-3300)");
  Serial.println("COUPLING <ch> <type> - Set coupling (AC/DC)");
  Serial.println("POSITION <ch> <value> - Set vertical position (-3 to +3 div)");
  Serial.println("TMODE <mode> - Set trigger mode (AUTO/NORM/SINGLE)");
  Serial.println("TSLOPE <slope> - Set trigger slope (RISE/FALL)");
  Serial.println("RUN/STOP - Start/stop acquisition");
  Serial.println("AUTOSET - Auto setup all channels");
  Serial.println("STATUS - Show current settings");
  Serial.println("HELP - Show this help");
}

void printStatus() {
  Serial.println("\nCurrent settings:");
  
  // Channel status
  for(int i = 0; i < 4; i++) {
    Serial.print("CH");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(config.channelEnabled[i] ? "ON" : "OFF");
    Serial.print(", ");
    Serial.print(config.vScale[i]);
    Serial.print("mV/div, ");
    Serial.print(config.coupling[i] == COUPLING_DC ? "DC" : "AC");
    Serial.print(", Pos:");
    Serial.print(config.position[i], 1);
    Serial.println();
  }
  
  // Time base
  Serial.print("Time scale: ");
  Serial.print(config.hScale, 1);
  Serial.println("µs/div");
  
  // Trigger
  Serial.print("Trigger: CH");
  Serial.print(config.triggerChannel);
  Serial.print(", ");
  Serial.print(config.triggerLevel);
  Serial.print("mV, ");
  Serial.print(config.triggerMode == TRIG_AUTO ? "AUTO" : 
              (config.triggerMode == TRIG_NORMAL ? "NORMAL" : "SINGLE"));
  Serial.print(", ");
  Serial.println(config.triggerSlope == TRIG_RISING ? "RISING" : "FALLING");
  
  // Acquisition
  Serial.println(config.running ? "Status: RUNNING" : "Status: STOPPED");
}