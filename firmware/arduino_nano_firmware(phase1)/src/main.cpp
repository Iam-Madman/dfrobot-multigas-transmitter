#include <SoftwareSerial.h>
#include <TM1637Display.h>
#include "DFRobot_MultiGasSensor.h"

// ==========================================
// PIN CONFIGURATION & INSTANCE DECLARATIONS
// ==========================================

// TM1637 4-Digit 7-Segment Display Configuration Pins
#define TM1637_CLK 3  // Changed from 4 to 3
#define TM1637_DIO 2  // Changed from 5 to 2
TM1637Display display(TM1637_CLK, TM1637_DIO);

// Sensor UART Configuration (Pins 2 and 3)
SoftwareSerial mySerial(11, 12); // Pin 11 is RX (Connects to Sensor TX), Pin 12 is TX (Connects to Sensor RX)
DFRobot_GAS_SoftWareUart gas(&mySerial);

// ==========================================
// CUSTOM 7-SEGMENT DISPLAY BITMASKS
// ==========================================

// Fixed 7-segment bitmask array to render the word "HEAt"
const uint8_t SEG_HEAT[] = { 
    SEG_B|SEG_C|SEG_E|SEG_F|SEG_G,       // H
    SEG_A|SEG_D|SEG_E|SEG_F|SEG_G,       // E
    SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G, // A
    SEG_D|SEG_E|SEG_F|SEG_G              // t 
};

// Fixed 7-segment bitmask array to render the word "Err " (Trailing blank space)
const uint8_t SEG_ERR[] = { 
    SEG_A|SEG_D|SEG_E|SEG_F|SEG_G,       // E
    SEG_E|SEG_G,                         // r
    SEG_E|SEG_G,                         // r
    0x00                                 // Blank space
};

// Fixed 7-segment bitmask array to render the word "Good"
const uint8_t SEG_GOOD[] = { 
    SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G, // G (Looks like a clean '9' or '6')
    SEG_C|SEG_D|SEG_E|SEG_G,             // o
    SEG_C|SEG_D|SEG_E|SEG_G,             // o
    SEG_B|SEG_C|SEG_D|SEG_E|SEG_G        // d (Lowercase prevents it looking like a '0')
};

// Spinning loading animation frames utilizing outer display segments
const uint8_t ANIM_FRAME_0[] = { SEG_A, SEG_A, SEG_A, SEG_A };       // Top bars active
const uint8_t ANIM_FRAME_1[] = { 0x00,  0x00,  0x00,  SEG_B|SEG_C }; // Right-most vertical bars active
const uint8_t ANIM_FRAME_2[] = { SEG_D, SEG_D, SEG_D, SEG_D };       // Bottom bars active
const uint8_t ANIM_FRAME_3[] = { SEG_E|SEG_F, 0x00,  0x00,  0x00 }; // Left-most vertical bars active

// ==========================================
// SYSTEM SETUP & INITIALIZATION
// ==========================================
void setup() {
  // Initialize hardware serial for system debugging at 115200 bps
  Serial.begin(115200);
  delay(1000); // Guard delay to allow Serial Monitor to properly bind
  Serial.println("\n=== SENSOR DEBUG BOOTUP ===");
  
  // Configure the 7-segment display to maximum brightness
  display.setBrightness(0x0f);

  // 1. Initialize sensor hardware matching your working code
  // Loop indefinitely until communication with the gas sensor shield is validated
  while (!gas.begin()) {
      Serial.println("NO Devices !");
      display.setSegments(SEG_ERR);
      delay(1000);
  }
  Serial.println("Sensor Library Init: GOOD!");
  display.clear();

  // 2. Set Temperature Compensation
  // Activates built-in algorithmic adjustments to gas readings based on ambient temperature changes
  gas.setTempCompensation(gas.ON);

  // 3. Set Acquire Mode to INITIATIVE
  // Instructs the sensor to stream data packets continuously over UART without needing polling requests
  gas.changeAcquireMode(gas.INITIATIVE);
  delay(1000);

  // --- PRE-HEAT SENSOR HEALTH CHECK ---
  // Validates if the sensor is actually pumping out readable, non-corrupted data streams before heating
  Serial.println("Checking sensor data stream...");
  bool sensorReady = false;
  unsigned long startCheckTime = millis();

  // Give the sensor up to 2000ms to provide its first valid data packet
  while (millis() - startCheckTime < 2000) {
    if (true == gas.dataIsAvailable()) {
      // Check if we got actual data (using your corruption filter logic)
      if (AllDataAnalysis.temp > -100.0 && AllDataAnalysis.gastype != "") {
        sensorReady = true;
        break; // Sensor is responding normally, exit the check loop
      }
    }
    delay(10); // Small yield to prevent choking the processor
  }

  // If the sensor data stream is non-responsive or permanently corrupted, trigger an automated software reset
  if (!sensorReady) {
    Serial.println("CRITICAL ERROR: Sensor failed health check! Resetting...");
    display.setSegments(SEG_ERR);
    delay(3000); // Leave "Err" on display long enough to be seen
    
    // Software Reset Execution: Points program counter to address 0, mimicking a physical hardware reset
    void (* resetFunc) (void) = 0; 
    resetFunc(); 
  }

  Serial.println("Sensor health check: PASSED.");
  // Flash "GOOD" onto the display module for user validation
  display.setSegments(SEG_GOOD);
  delay(3000); // Hold the GOOD message for 3 seconds
  display.clear();
  display.setSegments(SEG_HEAT);
  delay(3000);
  display.clear();
  
  // -------------------------------------
  // --- Safe Warm-up Timer Block (30 seconds) ---
  // Electrochemical gas sensors require a chemical stabilization / heating period for accurate data
  Serial.println("Starting sensor heating cycle...");
  int warmupSeconds = 30;

  // --- MANUAL TIMING CONFIGURATION (Must add up to exactly 10) ---
  // Modifying these bounds dictates the cycling sequence of the UI elements during the 30-second loop
  int animSeconds  = 3;  // How many seconds to run the spinning animation (Phases 0, 1, 2)
  int textSeconds  = 2;  // How many seconds to display the static "HEAt" text (Phases 3, 4)
  int countSeconds = 5;  // How many seconds to display the raw numerical countdown (Phases 5-9)
  // ---------------------------------------------------------------

  for (int i = warmupSeconds; i > 0; i--) {
    // FIX: Calculate phase using an increasing progression (0 to 9) instead of backwards
    // Evaluates the current step within a repeating 10-second window blocks
    int phase = (warmupSeconds - i) % 10;
    
    Serial.print("Heating... "); 
    Serial.print(i); 
    Serial.println("s remaining.");

    // Check if current step falls into the Animation window (First 3 seconds of the block)
    if (phase < animSeconds) {
      // PHASE 1: Spinning animation. Takes exactly 1 second to cycle through 4 frames (250ms * 4)
      for (int animStep = 0; animStep < 4; animStep++) {
        if (animStep == 0)      display.setSegments(ANIM_FRAME_0);
        else if (animStep == 1) display.setSegments(ANIM_FRAME_1);
        else if (animStep == 2) display.setSegments(ANIM_FRAME_2);
        else if (animStep == 3) display.setSegments(ANIM_FRAME_3);
        delay(250); 
      }
    } 
    // Check if current step falls into the Text flash window (Next 2 seconds of the block)
    else if (phase < (animSeconds + textSeconds)) {
      // PHASE 2: Static "HEAt" text (Blocks execution for exactly 1 second)
      display.setSegments(SEG_HEAT);
      delay(1000);
    } 
    // FIX: Explicitly use countSeconds to validate the final window (Remaining 5 seconds of the block)
    else if (phase < (animSeconds + textSeconds + countSeconds)) {
      // PHASE 3: Raw Numerical Countdown (Blocks execution for exactly 1 second)
      display.showNumberDec(i, false, 4, 0);
      delay(1000);
    }
    // Fallback redundancy to catch edge exceptions, ensuring total execution always yields exactly 1 second
    else {
      // PHASE 3: Raw Numerical Countdown
      display.showNumberDec(i, false, 4, 0);
      delay(1000); // 1 full second
    }
  }
  
  display.clear();
  Serial.println("Heating cycle complete.");
  
  // --- GOOD MESSAGE BANNER ---
  Serial.println("\n=================================");
  Serial.println("  SYSTEM READY: Sensor Warmup OK ");
  Serial.println("=================================\n");
  
  // Flash "GOOD" onto the display module for user validation
  display.setSegments(SEG_GOOD);
  delay(3000); // Hold the GOOD message for 3 seconds
  display.clear();

  // Clear any noise or fragmented serial packets accumulated in the software serial ring buffer during warmup delays
  while(mySerial.available() > 0) {
    mySerial.read();
  }
  Serial.println("Buffer flushed. Entering main loop...");
}

// ==========================================
// MAIN PROGRAM LOOP (DATA STREAMING)
// ==========================================
void loop() {
  // Check if a complete, newly arrived streaming packet frame has arrived over UART from the sensor shield
  if (true == gas.dataIsAvailable()) {
    
    // Extract parsed, cached telemetry structured variables calculated internally by the sensor library
    String gasType = AllDataAnalysis.gastype;
    float concentration = AllDataAnalysis.gasconcentration;
    float boardTemp = AllDataAnalysis.temp;

    // --- CORRUPTION FILTER ---
    // Instead of exiting via return;, we handle corruption gracefully 
    // by skipping the print, but NOT skipping the entire loop cycle.
    // Checks for uninitialized or invalid conditions (e.g., disconnected lines or noise reads)
    if (boardTemp > -100.0 && gasType != "" && gasType.length() > 0) {
      
      // 1. Print out valid telemetry to Serial Monitor
      Serial.println("========================");
      Serial.print("gastype: ");
      Serial.println(gasType);
      Serial.println("------------------------");
      Serial.print("gasconcentration: ");
      Serial.print(concentration);
      
      // Dynamic metric suffixing evaluation based on gas characteristics
      if (gasType.equals("O2")) {
        Serial.println(" %VOL"); // Oxygen uses Volume Percentage units
      } else {
        Serial.println(" PPM");  // Toxic gases/other variants use Parts Per Million units
      }
      
      Serial.println("------------------------");
      Serial.print("temp: ");
      Serial.print(boardTemp);
      Serial.println(" C");
      Serial.println("========================");

      // 2. Safely update TM1637 display with the freshly validated data
      // Boundary safety validation to handle sudden transmission anomalies or sensor errors 
      if (concentration < 0 || concentration > 9999) {
        display.setSegments(SEG_ERR);
      } else {
        // Mathematical rounding trick (+0.5 cast to int) to avoid truncation inaccuracies on float points
        int displayValue = (int)(concentration + 0.5); 
        display.showNumberDec(displayValue, false, 4, 0); // Update numeric value on display
      }
      
    } else {
      // Quietly consume any stray asynchronous bit corruptions without stalling execution or processing garbage data
      while(mySerial.available() > 0) {
        mySerial.read();
      }
    }
  }
  
  // Keep this delay small (around 10-20ms) or drop it completely. 
  // Large delays cause SoftwareSerial's internal 64-byte ring buffer to overflow, generating corrupted packets.
  delay(10); 
}