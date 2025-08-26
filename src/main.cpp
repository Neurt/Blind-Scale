#include <HX711_ADC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DFRobotDFPlayerMini.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <esp_bt.h>

// Timing variables
unsigned long lastDisplayTime = 0;
unsigned long lastBatteryCheck = 0;
unsigned long stableStartTime = 0;
unsigned long weightRecordedTime = 0;
unsigned long stepOffTime = 0;
unsigned long measurementStartTime = 0;  // Track when measurement started

// Pins
const uint8_t HX711_dout = 2;
const uint8_t HX711_sck = 33;
const uint8_t BATTERY_PIN = 34;

// Create objects
DFRobotDFPlayerMini myDFPlayer;
HardwareSerial playerSerial(1);
HX711_ADC loadCell(HX711_dout, HX711_sck);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// I2C pins
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

// Constants - grouped for better cache locality
const uint16_t SCALE_EEPROM_ADDR = 0;
const float WEIGHT_THRESHOLD = 1.0f;
const uint8_t READINGS_TO_AVERAGE = 5;
const uint16_t DISPLAY_REFRESH_MS = 150;
const uint8_t STABILITY_TOLERANCE = 25;
const float DISPLAY_CHANGE_THRESHOLD = 0.1f;
const uint16_t STABLE_DURATION_REQUIRED = 600;
const uint16_t WEIGHT_RESET_DELAY = 5000;
const uint16_t AUTO_TARE_DELAY = 2000;
const float AUTO_TARE_THRESHOLD = 10.0f;
const uint8_t STABILITY_COUNT_THRESHOLD = 2;
const uint16_t MEASUREMENT_TIMEOUT = 8000;  // Force output after 8 seconds if not stable

// Battery constants
const float BATTERY_DIVIDER_RATIO = 1.56f; 
const float INITIAL_BATTERY_VOLTAGE = 3.93f;
const uint8_t VOLTAGE_READINGS = 10;
const uint32_t BATTERY_CHECK_INTERVAL = 60000;     // Check battery every 60s
const uint32_t BATTERY_CHECK_IDLE_INTERVAL = 120000; // In idle state, check every 120s
const float LOW_BATTERY_THRESHOLD = 3.7f;

// Power saving constants
const uint32_t SLEEP_TIMEOUT = 60000;              // Sleep display after 60s
const uint32_t DEEP_SLEEP_TIMEOUT = 300000;        // Enter deep sleep after 5 minutes of inactivity
const uint8_t HX711_IDLE_RATE = 10;                // Reduce sample rate when idle
const int8_t CPU_FREQ_MHZ = 80;                    // Reduced CPU frequency (from default 240MHz)

// Optimized state variables - grouped by usage
struct ScaleState {
  float weightBuffer[READINGS_TO_AVERAGE];
  float lastDisplayedWeight;
  float recordedWeight;
  float lastBatteryVoltage;
  int8_t bufferIndex;
  int8_t stabilityCounter;
  bool lastStabilityState;
  bool weightRecorded;
  bool personOnScale;
  bool tareScheduled;
  bool newDataReady;
  bool isCharging;
  bool lowBatteryWarning;
  unsigned long lastLowBatteryAlert;  // New: timestamp of last low battery alert
  bool audioPlaying;                  // New: flag to track if audio is currently playing
} state = {0};

// Battery readings array - separate for better memory layout
float batteryReadings[VOLTAGE_READINGS];
int8_t batteryReadingIndex = 0;
bool batteryReadingsInitialized = false;

// Activity tracking
uint32_t lastActivityTime = 0;
bool displaySleeping = false;

// Function prototypes
void calibrate();
void displayWeight(float weight, bool isStable);
void displayMessage(const char* line1, const char* line2 = "", int delayMs = 0);
inline float calculateAverageWeight();
inline bool checkWeightStability(float currentWeight);
inline float roundToOneDecimal(float value);
void displayRecordedWeight(float weight);
void playRecordedWeight(float recordedWeight);
float checkBatteryAndCompensate(float weight);
void handleSerialCommands();
void updateWeightMeasurement();
void processWeightStability(float currentWeight);
void checkAndDisplayBatteryStatus();
void checkSleepMode();
void wakeDisplay();
void registerActivity();
float getBatteryVoltage();
void managePowerSaving();

void setup() {
  // Set CPU frequency to save power
  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  
  // Disable Bluetooth at startup to save power
  btStop();
  
  Serial.begin(115200);
  delay(10);
  
  // Initialize hardware with power-saving in mind
  pinMode(BATTERY_PIN, INPUT);
  
  // Initialize I2C with power saving
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // Lower I2C clock speed to 100kHz saves power
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  displayMessage("Scale Starting", "Please wait...");
  
  // Initialize scale with optimized timing
  loadCell.begin();
  loadCell.setSamplesInUse(1);  // Use fewer samples for faster response
  const unsigned long stabilizingTime = 1000; // Reduced time
  loadCell.start(stabilizingTime, false);
  
  if (loadCell.getTareTimeoutFlag() || loadCell.getSignalTimeoutFlag()) {
    displayMessage("Error: No scale", "Check connections");
    Serial.println("HX711 not found. Check wiring.");
    while (1); // In production, should enter low-power mode here
  }
  
  // Quick stabilization
  displayMessage("Stabilizing...", "Don't touch scale");
  for (int i = 0; i < 3; i++) { 
    loadCell.update();
    delay(30);
  }
  
  // Tare operation
  displayMessage("Taring...", "Don't touch scale");
  loadCell.tareNoDelay();
  while (!loadCell.getTareStatus()) {
    loadCell.update();
    delay(10);
  }
  
  // Initialize DFPlayer with power considerations
  playerSerial.begin(9600, SERIAL_8N1, 27, 14);
  if (!myDFPlayer.begin(playerSerial)) {
    Serial.println("Unable to begin DFPlayer");
  } else {
    Serial.println("DFPlayer Mini online");
    myDFPlayer.volume(29);
    
    // Play ready notification after tare is complete
    delay(500); // Small delay to ensure DFPlayer is ready
    myDFPlayer.play(001); 
    delay(2000); // Wait for the audio to finish
  }
  
  // Load calibration factor
  float calibrationFactor;
  #if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(512);
  #endif
  
  EEPROM.get(SCALE_EEPROM_ADDR, calibrationFactor);
  
  if (isnan(calibrationFactor) || calibrationFactor < 0.01f || calibrationFactor > 100000.0f) {
    calibrationFactor = 2553.0f;
    displayMessage("No calibration", "Using default", 1000);
  }
  
  loadCell.setCalFactor(calibrationFactor);
  
  // Only check battery at start
  checkBatteryAndCompensate(0);
  
  // Set to lower power sampling rate for idle state
  loadCell.setSamplesInUse(4);  // Use more samples for better stability when idle
  
  displayMessage("Ready to weigh", "Step on scale");
  
  // Initialize activity timer and other state variables
  lastActivityTime = millis();
  displaySleeping = false;
  state.lastLowBatteryAlert = 0;
  state.audioPlaying = false;
}

void loop() {
  // Update scale data
  if (loadCell.update()) {
    state.newDataReady = true;
    // Only register activity if weight changes significantly
    float currentReading = loadCell.getData();
    if (abs(currentReading) > 5000) { // ~5kg threshold
      registerActivity();
    }
  }
  
  // Handle serial commands
  handleSerialCommands();
  
  // Handle scheduled tare
  if (state.tareScheduled && millis() - stepOffTime > AUTO_TARE_DELAY) {
    state.tareScheduled = false;
    
    loadCell.update();
    float currentReading = loadCell.getData();
    
    if (abs(currentReading) < AUTO_TARE_THRESHOLD) {
      displayMessage("Auto-taring...", "Please wait");
      loadCell.tareNoDelay();
      
      while (!loadCell.getTareStatus()) {
        loadCell.update();
      }
      
      // Reset state
      memset(state.weightBuffer, 0, sizeof(state.weightBuffer));
      state.stabilityCounter = 0;
      state.lastDisplayedWeight = 0;
      state.bufferIndex = 0;
      
      displayMessage("Ready to weigh", "Step on scale");
      
      // Play auto-tare complete notification
      if (!state.audioPlaying) {
        state.audioPlaying = true;
        myDFPlayer.play(001); 
        delay(2000); // Wait for the audio to finish
        state.audioPlaying = false;
      }
    } else {
      displayMessage("Ready to weigh", "Step on scale");
    }
  }
  
  // Update weight measurement
  updateWeightMeasurement();
  
  // Check battery status less frequently
  unsigned long now = millis();
  if (now - lastBatteryCheck > 
      (state.personOnScale ? BATTERY_CHECK_INTERVAL : BATTERY_CHECK_IDLE_INTERVAL)) {
    checkAndDisplayBatteryStatus();
  }
  
  // Check and handle sleep modes
  checkSleepMode();
  
  // Manage power saving features
  managePowerSaving();
  
  // Short delay when idle to reduce power consumption
  if (!state.personOnScale && !state.newDataReady) {
    delay(20); // Small delay when idle
  }
}

void managePowerSaving() {
  // Adjust HX711 sampling rate based on activity
  static bool highSamplingRate = true;
  static int sampleCounter = 0;
  
  if (state.personOnScale && !highSamplingRate) {
    loadCell.setSamplesInUse(1);
    highSamplingRate = true;
  } 
  else if (!state.personOnScale && highSamplingRate) {
    // Reduce sampling rate when idle
    sampleCounter++;
    if (sampleCounter > HX711_IDLE_RATE) {
      loadCell.setSamplesInUse(4);
      highSamplingRate = false;
      sampleCounter = 0;
    }
  }
  
  // Check for deep sleep condition
  if (millis() - lastActivityTime > DEEP_SLEEP_TIMEOUT && !state.personOnScale) {
    Serial.println("Entering deep sleep mode...");
    displayMessage("Power saving", "Entering sleep...");
    delay(1000);
    
    // Prepare for deep sleep
    lcd.noBacklight();
    
    // Configure wake sources
    esp_sleep_enable_ext0_wakeup((gpio_num_t)HX711_dout, 0);
    esp_sleep_enable_timer_wakeup(30000000); // 30 seconds backup timer
    
    // Enter deep sleep
    esp_deep_sleep_start();
  }
}

void checkSleepMode() {
  // Check if it's time to sleep display
  if (!displaySleeping && (millis() - lastActivityTime > SLEEP_TIMEOUT) && !state.personOnScale) {
    // No activity for a while, put display to sleep
    lcd.noBacklight();
    displaySleeping = true;
    Serial.println("Display going to sleep due to inactivity");
  }
}

void wakeDisplay() {
  if (displaySleeping) {
    lcd.backlight();
    displaySleeping = false;
    Serial.println("Display woken up");
  }
}

void registerActivity() {
  lastActivityTime = millis();
  if (displaySleeping) {
    wakeDisplay();
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    registerActivity(); // Any serial command is activity
    char inByte = Serial.read();
    switch (inByte) {
      case 't':
        displayMessage("Taring...");
        loadCell.tareNoDelay();
        while (!loadCell.getTareStatus()) {
          loadCell.update();
        }
        // Reset state
        memset(state.weightBuffer, 0, sizeof(state.weightBuffer));
        state.stabilityCounter = 0;
        state.lastDisplayedWeight = 0;
        state.bufferIndex = 0;
        displayMessage("Tare complete", "Step on scale", 1000);
        
        // Play tare complete notification
        state.audioPlaying = true;
        myDFPlayer.play(001); 
        delay(2000); // Wait for the audio to finish
        state.audioPlaying = false;
        break;
        
      case 'c':
        calibrate();
        break;
        
      case 'r':
        state.weightRecorded = false;
        state.personOnScale = false;
        state.stabilityCounter = 0;
        stableStartTime = 0;
        displayMessage("Measurement reset", "Step on scale", 1000);
        break;
        
      case 'v': // Battery voltage display
        {
          Serial.print("Battery voltage: ");
          Serial.print(state.lastBatteryVoltage, 2);
          Serial.println("V");
        }
        break;
        
      case 'b': // Battery calibration
        {
          Serial.println("Battery calibration mode");
          Serial.println("Enter actual battery voltage from multimeter (e.g. 3.93): ");
          
          String inputValue = "";
          bool valueEntered = false;
          
          while (!valueEntered) {
            if (Serial.available()) {
              char c = Serial.read();
              
              if (c == 8 || c == 127) { // Backspace or Delete
                if (inputValue.length() > 0) {
                  inputValue.remove(inputValue.length() - 1);
                  Serial.print("\b \b");
                }
              }
              else if (c == '\n' || c == '\r') {
                float actualVoltage = inputValue.toFloat();
                
                if (actualVoltage > 0) {
                  float rawADC = analogRead(BATTERY_PIN) * 3.3f / 4095.0f;
                  float newRatio = actualVoltage / rawADC;
                  
                  Serial.print("New battery divider ratio: ");
                  Serial.println(newRatio, 4);
                  Serial.println("Update your code with this new value for BATTERY_DIVIDER_RATIO");
                  
                  const_cast<float&>(BATTERY_DIVIDER_RATIO) = newRatio;
                  lastBatteryCheck = 0;
                  checkBatteryAndCompensate(0);
                  
                  valueEntered = true;
                } else {
                  Serial.println("Invalid voltage. Try again:");
                  inputValue = "";
                }
              }
              else if (isDigit(c) || c == '.') {
                inputValue += c;
                Serial.print(c);
              }
            }
            delay(10);
          }
        }
        break;
        
      case '+':
        {
          uint8_t currentVolume = myDFPlayer.readVolume();
          if (currentVolume < 30) {
            myDFPlayer.volume(currentVolume + 1);
            Serial.print("Volume increased to: ");
            Serial.println(currentVolume + 1);
          } else {
            Serial.println("Volume already at maximum (30)");
          }
        }
        break;
          
      case '-':
        {
          uint8_t currentVolume = myDFPlayer.readVolume();
          if (currentVolume > 0) {
            myDFPlayer.volume(currentVolume - 1);
            Serial.print("Volume decreased to: ");
            Serial.println(currentVolume - 1);
          } else {
            Serial.println("Volume already at minimum (0)");
          }
        }
        break;
          
      case 'l':
        {
          uint8_t currentVolume = myDFPlayer.readVolume();
          Serial.print("Current volume: ");
          Serial.print(currentVolume);
          Serial.println("/30");
        }
        break;
        
      case 's': // Power stats
        {
          Serial.println("Power saving statistics:");
          Serial.print("CPU frequency: ");
          Serial.print(getCpuFrequencyMhz());
          Serial.println(" MHz");
          Serial.print("WiFi status: ");
          Serial.println(state.wifiConnected ? "Connected" : "Disconnected");
          Serial.print("Display status: ");
          Serial.println(displaySleeping ? "Sleep" : "Active");
          Serial.print("Inactivity time: ");
          Serial.print((millis() - lastActivityTime) / 1000);
          Serial.println(" seconds");
        }
        break;
    }
  }
}

void updateWeightMeasurement() {
  if (!state.newDataReady || millis() - lastDisplayTime < DISPLAY_REFRESH_MS) {
    return;
  }
  
  state.newDataReady = false;
  
  // Get and process weight
  float rawWeight = loadCell.getData();
  float currentWeight = roundToOneDecimal(rawWeight / 1000.0f);
  currentWeight = checkBatteryAndCompensate(currentWeight);
  
  // Register activity when weight changes significantly
  if (abs(currentWeight - state.lastDisplayedWeight) > WEIGHT_THRESHOLD) {
    registerActivity();
  }
  
  // Update buffer
  state.weightBuffer[state.bufferIndex] = currentWeight;
  state.bufferIndex = (state.bufferIndex + 1) % READINGS_TO_AVERAGE;
  
  // State transitions
  if (!state.personOnScale && currentWeight > WEIGHT_THRESHOLD) {
    state.personOnScale = true;
    state.tareScheduled = false;
    state.stabilityCounter = 0;
    state.weightRecorded = false;
    measurementStartTime = millis();  // Start timeout timer
    loadCell.setSamplesInUse(1);
    displayMessage("Measuring...");
  }
  
  // Reset measurement if needed
  if (state.personOnScale && state.weightRecorded && 
      (millis() - weightRecordedTime > WEIGHT_RESET_DELAY)) {
    state.weightRecorded = false;
    state.stabilityCounter = 0;
    stableStartTime = 0;
    displayMessage("New measurement", "Stand still...");
    
    // Fill buffer with current weight (optimized for less iterations)
    for (uint8_t i = 0; i < READINGS_TO_AVERAGE; i++) {
      state.weightBuffer[i] = currentWeight;
    }
  }
  
  // Process weight when person is on scale
  if (state.personOnScale && !state.weightRecorded) {
    processWeightStability(currentWeight);
  }
  
  // Handle person stepping off
  if (state.personOnScale && currentWeight < WEIGHT_THRESHOLD) {
    state.personOnScale = false;
    state.stabilityCounter = 0;
    stableStartTime = 0;
    stepOffTime = millis();
    state.tareScheduled = true;
    displayMessage("Processing...", "Please wait");
    
    // Play ready notification when person steps off (after a short delay)
    delay(1000); // Wait a moment after stepping off
    if (!state.audioPlaying) {
      state.audioPlaying = true;
      myDFPlayer.play(001); 
      delay(2000); // Wait for the audio to finish
      state.audioPlaying = false;
    }
  }
  
  lastDisplayTime = millis();
}

void processWeightStability(float currentWeight) {
  bool isStable = checkWeightStability(currentWeight);
  
  // Track stable duration
  if (isStable && stableStartTime == 0) {
    stableStartTime = millis();
    registerActivity(); // Stability change is activity
  } else if (!isStable) {
    stableStartTime = 0;
  }
  
  // Update display if needed (throttle updates to save power)
  if (abs(currentWeight - state.lastDisplayedWeight) > DISPLAY_CHANGE_THRESHOLD || 
      isStable != state.lastStabilityState) {
    
    float weightToDisplay = isStable ? roundToOneDecimal(calculateAverageWeight()) : currentWeight;
    displayWeight(weightToDisplay, isStable);
    
    state.lastDisplayedWeight = weightToDisplay;
    state.lastStabilityState = isStable;
  }
  
  // Record weight if stable long enough OR if timeout reached
  bool shouldRecord = false;
  bool timeoutReached = (millis() - measurementStartTime) > MEASUREMENT_TIMEOUT;
  
  if (isStable && (millis() - stableStartTime) > STABLE_DURATION_REQUIRED) {
    shouldRecord = true;
    Serial.println("Weight recorded - STABLE reading");
  } else if (timeoutReached && !state.weightRecorded) {
    shouldRecord = true;
    Serial.println("Weight recorded - TIMEOUT reached (may be unstable)");
    displayMessage("Timeout reached", "Recording weight...", 1000);
  }
  
  if (shouldRecord) {
    state.recordedWeight = roundToOneDecimal(calculateAverageWeight());
    state.weightRecorded = true;
    weightRecordedTime = millis();
    
    // 1. Display weight on LCD first
    displayRecordedWeight(state.recordedWeight);
    Serial.print("WEIGHT RECORDED: ");
    Serial.println(state.recordedWeight);
    
    // 2. Play the audio output immediately
    playRecordedWeight(state.recordedWeight);
    
    // 3. Show recorded weight and play ready notification
    delay(1000); // Brief pause
    displayMessage("Weight recorded", "Ready for next", 2000);
    
    // Play ready notification
    if (!state.audioPlaying) {
      state.audioPlaying = true;
      myDFPlayer.play(001); 
      delay(2000); // Wait for the audio to finish
      state.audioPlaying = false;
    }
    
    // Return to ready state
    displayMessage("Ready to weigh", "Step on scale");
  }
}

float getBatteryVoltage() {
  // More efficient battery reading
  const uint8_t readings = 5;
  float sum = 0;
  
  for (uint8_t i = 0; i < readings; i++) {
    sum += analogRead(BATTERY_PIN);
  }
  
  // Average and convert in one step
  return (sum / readings) * 3.3f / 4095.0f * BATTERY_DIVIDER_RATIO;
}

float checkBatteryAndCompensate(float weight) {
  static float lastVoltageReading = 0;
  static unsigned long lastChargingCheck = 0;
  const float CHARGING_THRESHOLD = 0.05f;
  const unsigned long CHARGING_CHECK_INTERVAL = 10000;
  
  // Only check battery at intervals - extended for power saving
  if (millis() - lastBatteryCheck < 
      (state.personOnScale ? BATTERY_CHECK_INTERVAL : BATTERY_CHECK_IDLE_INTERVAL) 
      && batteryReadingsInitialized) {
    
    // Check for charging status less frequently
    if (millis() - lastChargingCheck > CHARGING_CHECK_INTERVAL) {
      // Detect charging (voltage increasing)
      if (state.lastBatteryVoltage > (lastVoltageReading + CHARGING_THRESHOLD)) {
        state.isCharging = true;
      } else if (state.isCharging && state.lastBatteryVoltage < 4.1f) {
        state.isCharging = false;
      }
      
      lastVoltageReading = state.lastBatteryVoltage;
      lastChargingCheck = millis();
    }
    
    if (state.lastBatteryVoltage > 0) {
      // Optimize calculation to avoid division
      float voltageRatio = INITIAL_BATTERY_VOLTAGE / state.lastBatteryVoltage;
      return weight * (1.0f + (voltageRatio - 1.0f) * 0.2f);
    }
    return weight;
  }
  
  lastBatteryCheck = millis();
  
  // Read voltage less frequently to save power
  float batteryVoltage = getBatteryVoltage();
  
  batteryReadings[batteryReadingIndex] = batteryVoltage;
  batteryReadingIndex = (batteryReadingIndex + 1) % VOLTAGE_READINGS;
  
  // Calculate average
  float avgVoltage = 0;
  int readingsToUse = batteryReadingsInitialized ? VOLTAGE_READINGS : batteryReadingIndex;
  if (readingsToUse > 0) {  // Prevent division by zero
    for (int i = 0; i < readingsToUse; i++) {
      avgVoltage += batteryReadings[i];
    }
    avgVoltage /= readingsToUse;
  }
  state.lastBatteryVoltage = avgVoltage;
  
  // Check for charging (voltage increasing significantly)
  if (avgVoltage > (lastVoltageReading + CHARGING_THRESHOLD)) {
    state.isCharging = true;
  } else if (state.isCharging && avgVoltage < 4.1f) {
    state.isCharging = false;
  }
  lastVoltageReading = avgVoltage;
  lastChargingCheck = millis();
  
  if (batteryReadingIndex == 0) {
    batteryReadingsInitialized = true;
  }
  
  // Low battery warning
  state.lowBatteryWarning = (avgVoltage < LOW_BATTERY_THRESHOLD);
  if (state.lowBatteryWarning) {
    Serial.print("WARNING: Low battery - ");
    Serial.print(avgVoltage, 2);
    Serial.println("V");
  }
  
  // Apply compensation
  float voltageRatio = INITIAL_BATTERY_VOLTAGE / max(avgVoltage, 3.0f);
  return weight * (1.0f + (voltageRatio - 1.0f) * 0.2f);
}

void checkAndDisplayBatteryStatus() {
  static unsigned long lastBatteryDisplayCheck = 0;
  const unsigned long BATTERY_DISPLAY_INTERVAL = 30000; // Every 30 seconds
  const unsigned long LOW_BATTERY_ALERT_INTERVAL = 300000; // Play alert every 5 minutes max
  
  if (millis() - lastBatteryDisplayCheck > BATTERY_DISPLAY_INTERVAL) {
    lastBatteryDisplayCheck = millis();
    
    if (!state.personOnScale) {
      // Show battery voltage in serial monitor (don't log too much to save power)
      Serial.print("Battery: ");
      Serial.print(state.lastBatteryVoltage, 2);
      Serial.println("V");
      
      // Only show full screen status for low battery
      if (state.lowBatteryWarning) {
        char batteryInfo[16];
        sprintf(batteryInfo, "%.2fV (Low!)", state.lastBatteryVoltage);
        displayMessage("Battery Low!", batteryInfo, 2000);
        
        // Play audio alert if:
        // 1. It hasn't been played recently
        // 2. No audio is currently playing (weight announcement)
        // 3. No person is on the scale (to avoid interrupting measurement)
        if (!state.audioPlaying && 
            millis() - state.lastLowBatteryAlert > LOW_BATTERY_ALERT_INTERVAL) {
          Serial.println("Playing low battery alert sound");
          
          state.audioPlaying = true;
          myDFPlayer.play(202); // Play the 202.mp3 file for low battery alert
          state.lastLowBatteryAlert = millis();
          delay(2000); // Wait for audio to finish
          state.audioPlaying = false;
        }
        
        // Return to ready state message
        displayMessage("Ready to weigh", "Step on scale");
      }
    }
  }
}

// Optimized inline functions
inline float roundToOneDecimal(float value) {
  return round(value * 10.0f) / 10.0f;
}

inline float calculateAverageWeight() {
  float sum = 0;
  for (int i = 0; i < READINGS_TO_AVERAGE; i++) {
    sum += state.weightBuffer[i];
  }
  return sum / READINGS_TO_AVERAGE;
}

inline bool checkWeightStability(float currentWeight) {
  float sumDiff = 0;
  for (int i = 0; i < READINGS_TO_AVERAGE; i++) {
    sumDiff += abs(state.weightBuffer[i] - currentWeight) * 1000.0f;
  }
  
  if (sumDiff / READINGS_TO_AVERAGE > STABILITY_TOLERANCE) {
    state.stabilityCounter = 0;
    return false;
  }
  
  state.stabilityCounter++;
  return state.stabilityCounter >= STABILITY_COUNT_THRESHOLD;
}

void displayWeight(float weight, bool isStable) {
  // Cache the string to prevent repeated operations
  static char weightStrCache[10] = {0};
  static float lastWeight = -999.0f;
  
  // Only update string if weight changed significantly
  if (abs(weight - lastWeight) > 0.05f) {
    dtostrf(weight, 5, 1, weightStrCache);
    lastWeight = weight;
  }
  
  lcd.setCursor(0, 0);
  lcd.print("Weight: ");
  lcd.print(weightStrCache);
  lcd.print("Kg");
  
  // Update second line only when stability changes to save power
  static bool lastIsStable = false;
  if (isStable != lastIsStable) {
    lcd.setCursor(0, 1);
    lcd.print("                ");  // Clear line
    lcd.setCursor(0, 1);
    
    if (isStable) {
      lcd.print("Reading stable");
    } else {
      lcd.print("Stabilizing ");
      for (int i = 0; i < state.stabilityCounter; i++) {
        lcd.print(".");
      }
    }
    lastIsStable = isStable;
  }
}

void displayRecordedWeight(float weight) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RECORDED WEIGHT:");
  
  char weightStr[10];
  dtostrf(weight, 5, 1, weightStr);
  
  lcd.setCursor(0, 1);
  lcd.print(weightStr);
  lcd.print(" Kg");
  lcd.print(" [SENT]");
}

void displayMessage(const char* line1, const char* line2, int delayMs) {
  registerActivity(); // Display update is activity
  lcd.clear();
  lcd.print(line1);
  
  if (strlen(line2) > 0) {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
  
  if (delayMs > 0) {
    delay(delayMs);
  }
}

void playRecordedWeight(float recordedWeight) {
  state.audioPlaying = true;
  
  int integerPart = (int)recordedWeight;
  float decimalPart = recordedWeight - integerPart;
  int decimalDigit = (int)round(decimalPart * 10.0f);

  myDFPlayer.play(integerPart);
  delay(1800);
  
  if (decimalDigit > 0 && decimalDigit <= 9) {
    int decimalIndex = 201 + decimalDigit;
    myDFPlayer.play(decimalIndex);
    delay(1200);
  }

  myDFPlayer.play(201); // "kilogram"
  delay(1500); // Wait for "kilogram" to finish
  
  state.audioPlaying = false;
}

void calibrate() {
  displayMessage("Calibration Mode", "Remove all weight");
  Serial.println("Calibration started - Remove weight, send 't' to tare");
  
  boolean _resume = false;
  while (!_resume) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') {
        loadCell.tareNoDelay();
        _resume = true;
      }
    }
    loadCell.update();
    if (loadCell.getTareStatus()) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }
  
  displayMessage("Calibration Mode", "Place known weight");
  Serial.println("Enter weight in kg and press Enter:");
  
  float knownWeight = 0;
  String inputString = "";
  
  while (knownWeight <= 0) {
    if (Serial.available() > 0) {
      char inChar = Serial.read();
      
      if (isDigit(inChar) || inChar == '.') {
        inputString += inChar;
      } else if (inChar == '\n' || inChar == '\r') {
        knownWeight = inputString.toFloat();
        if (knownWeight <= 0) {
          displayMessage("Invalid weight", "Try again");
          inputString = "";
        }
      }
    }
    
    lcd.setCursor(0, 1);
    lcd.print("Weight: ");
    lcd.print(inputString);
    lcd.print("    ");
    delay(100);
    loadCell.update();
  }
  
  displayMessage("Don't move scale", "Taking readings...");
  delay(1000);
  
  loadCell.update();
  loadCell.refreshDataSet();
  
  float newCalibrationFactor = loadCell.getNewCalibration(knownWeight);
  loadCell.setCalFactor(newCalibrationFactor);
  
  #if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(512);
  #endif
  EEPROM.put(SCALE_EEPROM_ADDR, newCalibrationFactor);
  #if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
  #endif
  
  Serial.print("New calibration factor: ");
  Serial.println(newCalibrationFactor, 3);
  
  displayMessage("Calibration done", "Factor saved", 2000);
  displayMessage("Ready to weigh", "Step on scale");
}