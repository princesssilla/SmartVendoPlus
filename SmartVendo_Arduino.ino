#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include "HX711.h"



int plastictimeout = 10000; //How long the system will wait for bottle to measure plastic input weight
const unsigned long PLASTIC_SORTER_HOLD_TIME = 4000; // hold time valid/invalid position 
const unsigned long PAPER_RESPONSE_TIMEOUT = 10000; // 10 secs timeout (IR)
const unsigned long PLASTIC_GATE_TIMEOUT = 10000;   // 10 secs timeout (Ultrasonic)
int rollonTimeout = 30000;  //paper
int rollbackTimeout = 10000; //paper
const unsigned long BOTTLE_MOUNT_CONFIRM_TIME = 2000; 
int sorterangle = 100; //angle of sorter
bool paperPositionReached = false;


// RFID
#define SS_PIN 53    // SDA pin
#define RST_PIN 49   // RST pin

// START LIGHT
#define PAPER_RELAY_PIN 32
#define PLASTIC_RELAY_PIN 33
#define FAN_RELAY_PIN 34

// START SHREDDING
#define RELAY_PIN 35
// SHREDDER RELAY
#define SERVO_PIN 36
Servo shredderServo;

// REDEEM SERVOS
#define REDEEM_SERVO_PIN_1 37  // Slot 1
#define REDEEM_SERVO_PIN_2 38  // Slot 2
#define REDEEM_SERVO_PIN_3 39  // Slot 3
#define IR_SENSOR_PIN 46       // IR sensor for redeem slots

// PAPER VALIDATION SYSTEM 
#define PAPER_INSERT_IR_PIN 44    // IR sensor to detect paper insertion
#define PAPER_ROLLER_LEFT_PIN 42  // roller
#define PAPER_ROLLER_RIGHT_PIN 43 // roller
#define PAPER_POSITION_IR_PIN 45  // IR sensor to detect when paper is positioned for scanning

// PLASTIC VALIDATION SYSTEM
#define PLASTIC_GATE_SERVO_PIN 40 // gate
#define PLASTIC_SORT_SERVO_PIN 41 // sorter
#define HX711_DAT_PIN A2          // HX711 data pin
#define HX711_CLK_PIN A3          // HX711 clock pin
// SERVO MOTOR ANGLE FOR PLASTIC BOTTLE
int moveINVALID = 170; // ANGLE FOR VALID BOTTLE SERVO
int moveVALID = 50;    // ANGLE FOR INVALID SERVO

// Bottle mounting detection ultrasonic sensor
#define TRIG_BOTTLE_MOUNT 22
#define ECHO_BOTTLE_MOUNT 23
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BOTTLE_MOUNT_DISTANCE_MIN 2.0    // Minimum distance for bottle detection 
#define BOTTLE_MOUNT_DISTANCE_MAX 25.0    // Maximum distance for bottle detection
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BOTTLE_MOUNT_STABLE_COUNT 5      
#define BOTTLE_MOUNT_CHECK_INTERVAL 100  

// Ultrasonic sensors for bin monitoring
#define TRIG_PAPER 26
#define ECHO_PAPER 27
#define TRIG_PLASTIC 28
#define ECHO_PLASTIC 29
#define TRIG_SHREDDER 30
#define ECHO_SHREDDER 31

// for Ultrasonic full detection
#define FULL_THRESHOLD_BOTTLE 35.0
#define FULL_THRESHOLD_PAPER 15.0
#define CHECK_INTERVAL 10000  // how long our ultra sonic sensor need to detect item para mag send ng FULL(BIN)

// Water detection threshold for HX711 
#define WATER_DETECTION_THRESHOLD 25.0

MFRC522 mfrc522(SS_PIN, RST_PIN);  
String currentMaterial = "";       

// Servos for redeem slots
Servo redeemServo1;
Servo redeemServo2;
Servo redeemServo3;

// Servos for paper validation
Servo paperRollerLeft;
Servo paperRollerRight;

// Servos for plastic validation
Servo plasticGateServo;
Servo plasticSortServo;

// HX711 load cell for water detection
HX711 scale;

// Variables for paper validation
bool waitingForPaperInsert = false;
bool paperRollingInProgress = false;
bool waitingForPaperResponse = false;
bool paperAtPosition = false;  
unsigned long paperRollStartTime = 0;
unsigned long paperRollDuration = 0;
unsigned long paperResponseStartTime = 0;
bool rollInDirection = true;  

bool waitingForPlasticInsert = false;
bool plasticGateOpen = false;
bool waitingForPlasticResponse = false;
unsigned long plasticGateOpenTime = 0;
unsigned long plasticResponseStartTime = 0;

// Variables for bottle mounting detection
bool bottleMounted = false;
bool bottleMountingDetected = false;
unsigned long bottleMountDetectedTime = 0;

int stableBottleCount = 0;

bool waterDetected = false;

// Variables for plastic sorter
bool plasticSorterMoving = false;
bool plasticSorterHolding = false;
int plasticSorterTargetAngle = 90;
unsigned long plasticSorterHoldStartTime = 0;


// Bin status variables
bool paperBinFull = false;
bool plasticBinFull = false;
bool shredderBinFull = false;
unsigned long lastBinCheck = 0;

// Auto shredd
int shreddcount = 0;

// ====== NEW: Load Cell State Machine ======
enum LoadCellState {
  LC_IDLE,
  LC_WAITING_FOR_BOTTLE,
  LC_BOTTLE_MOUNTED,
  LC_TAKING_READINGS,
  LC_READINGS_COMPLETE
};

LoadCellState loadCellState = LC_IDLE;
unsigned long loadCellStateTime = 0;
const unsigned long LOADCELL_SETTLE_TIME = 500;    // Wait 1s after gate closes before taring
const unsigned long LOADCELL_TARE_TIME = 500;       // Time for taring process
const unsigned long LOADCELL_READING_TIME = 2000;   // Time to take weight readings

// ultrasonic sensor distance measurement
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = duration * 0.0343 / 2;
  
  if (duration == 0 || distance > 200) {
    return 200.0;
  }
  
  return distance;
}

// check bottle mounting
bool checkBottleMounted() {
  static unsigned long lastBottleCheck = 0;
  static float lastDistance = 200.0;
  
  if (millis() - lastBottleCheck < BOTTLE_MOUNT_CHECK_INTERVAL) {
    return bottleMountingDetected;
  }
  
  lastBottleCheck = millis();
  float distance = measureDistance(TRIG_BOTTLE_MOUNT, ECHO_BOTTLE_MOUNT);
  
  // Print distance for debugging
  Serial.print("Bottle Mount Distance: ");
  Serial.print(distance);
  
  // Check if distance is within bottle mounting range
  if (distance >= BOTTLE_MOUNT_DISTANCE_MIN && distance <= BOTTLE_MOUNT_DISTANCE_MAX) {
    // Check for stable reading 
    if (abs(distance - lastDistance) < 2.0) {
      stableBottleCount++;
    } else {
      stableBottleCount = 0;
    }
    
    lastDistance = distance;
    
    // If we have enough stable readings, bottle is mounted
    if (stableBottleCount >= BOTTLE_MOUNT_STABLE_COUNT && !bottleMountingDetected) {
      bottleMountingDetected = true;
      bottleMountDetectedTime = millis();
      Serial.println("BOTTLE MOUNTED");
    }
  } 
  
  else {
    // Bottle not detected or out of range
    stableBottleCount = 0;
    if (bottleMountingDetected) {
      bottleMountingDetected = false;
      Serial.println("BOTTLE NOT MOUNTED");
    }
  }
  
  return bottleMountingDetected;
}

// Function to check all bins and send status
void checkBins() {
  float paperDist = measureDistance(TRIG_PAPER, ECHO_PAPER);
  float plasticDist = measureDistance(TRIG_PLASTIC, ECHO_PLASTIC);
  float shredderDist = measureDistance(TRIG_SHREDDER, ECHO_SHREDDER);
  

  
  // Check paper bin
  if (paperDist <= FULL_THRESHOLD_PAPER && !paperBinFull) {
    Serial2.println("PAPER_BIN:FULL");

    paperBinFull = true;
  } else if (paperDist > FULL_THRESHOLD_PAPER && paperBinFull) {
    Serial2.println("PAPER_BIN:OK");

    paperBinFull = false;
  }
  
  // Check plastic bin
  if (plasticDist <= FULL_THRESHOLD_BOTTLE && !plasticBinFull) {
    Serial2.println("PLASTIC_BIN:FULL");

    plasticBinFull = true;
  } else if (plasticDist > FULL_THRESHOLD_BOTTLE && plasticBinFull) {
    Serial2.println("PLASTIC_BIN:OK");

    plasticBinFull = false;
  }
  
  // Check shredder bin
  if (shredderDist <= FULL_THRESHOLD_PAPER && !shredderBinFull) {
    Serial2.println("SHREDDER_BIN:FULL");

    shredderBinFull = true;
  } else if (shredderDist > FULL_THRESHOLD_PAPER && shredderBinFull) {
    Serial2.println("SHREDDER_BIN:OK");

    shredderBinFull = false;
  }
}

// Function to rotate a servo and check IR sensor for redeem
bool rotateAndCheckServo(Servo &servo, int startAngle, int endAngle, unsigned long rotationTime) {
    bool itemDropped = false;
    unsigned long startTime = millis();
    int currentPos = startAngle;
    
    while (millis() - startTime < rotationTime) {
        if (digitalRead(IR_SENSOR_PIN) == LOW) {
            Serial.println("Item detected by IR sensor!");
            itemDropped = true;
            break;
        }
        
        unsigned long elapsed = millis() - startTime;
        currentPos = map(elapsed, 0, rotationTime, startAngle, endAngle);
        currentPos = constrain(currentPos, startAngle, endAngle);
        servo.write(currentPos);
        delay(50);
    }
    
    servo.write(startAngle);
    return itemDropped;
}

// Function to control paper rollers
void startPaperRollers(bool rollIn) {
    if (rollIn) {
        // Roll in: left roller counter-clockwise, right roller clockwise
        paperRollerLeft.write(0);    // 0 = full speed counter-clockwise
        paperRollerRight.write(180); // 180 = full speed clockwise

    } else {
        // Roll back: opposite direction
        paperRollerLeft.write(180);  // 180 = full speed clockwise
        paperRollerRight.write(0);   // 0 = full speed counter-clockwise

    }
    
    paperRollStartTime = millis();
    paperRollingInProgress = true;
    rollInDirection = rollIn;
}

// Function to stop paper rollers
void stopPaperRollers() {
    paperRollerLeft.write(90);   // 90 = stop
    paperRollerRight.write(90);  // 90 = stop
    paperRollingInProgress = false;
    Serial.println("Rollers stopped");
}

const unsigned long WEIGHT_SAMPLE_INTERVAL = 120; 
unsigned long lastWeightSampleTime = 0;
const int WEIGHT_BUFFER_SIZE = 6;
float weightBuffer[WEIGHT_BUFFER_SIZE];
int weightBufferIndex = 0;
bool weightBufferFilled = false;

void updateLoadCellStateMachine() {
  switch (loadCellState) {
    case LC_IDLE:
      // Do nothing, load cell is idle
      break;
      
    case LC_WAITING_FOR_BOTTLE:
      // Just wait for bottle to be detected (handled in main loop)
      break;
      
    case LC_BOTTLE_MOUNTED:
      // Wait for gate to settle after closing
      if (millis() - loadCellStateTime >= LOADCELL_SETTLE_TIME) {
        
        loadCellState = LC_TAKING_READINGS;
        loadCellStateTime = millis();
        
        // Clear weight buffer
        for (int i = 0; i < WEIGHT_BUFFER_SIZE; i++) {
          weightBuffer[i] = 0.0;
        }
        weightBufferIndex = 0;
        weightBufferFilled = false;
        waterDetected = false;
      }
      break;
      
    case LC_TAKING_READINGS:
      // Take weight readings for specified time
      if (millis() - loadCellStateTime < LOADCELL_READING_TIME) {
        // Take weight samples
        if (millis() - lastWeightSampleTime >= WEIGHT_SAMPLE_INTERVAL) {
          lastWeightSampleTime = millis();
          if (scale.is_ready()) {
            
            float single = scale.get_units(1); 
            weightBuffer[weightBufferIndex++] = single;
            if (weightBufferIndex >= WEIGHT_BUFFER_SIZE) {
              weightBufferIndex = 0;
              weightBufferFilled = true;
            }
            
            // Calculate average
            int count = weightBufferFilled ? WEIGHT_BUFFER_SIZE : weightBufferIndex;
            if (count > 0) {
              float sum = 0.0;
              for (int i = 0; i < count; i++) sum += weightBuffer[i];
              float avg = sum / count;

              
              if (avg > WATER_DETECTION_THRESHOLD) {
                waterDetected = true;
                Serial.print("Water detected! Bottle weight: ");
                Serial.println(avg);
              }
            }
          }
        }
      } else {
        // Reading time complete
        loadCellState = LC_READINGS_COMPLETE;
        Serial.println("Load cell: Readings complete");
        
        // If water detected, handle it immediately
        if (waterDetected) {

          delay(2000);
          Serial2.println("INVALID:PLASTIC");
          
          movePlasticSorter(moveINVALID);
          digitalWrite(PLASTIC_RELAY_PIN, HIGH);
          currentMaterial = "";
          waitingForPlasticInsert = false;
          waitingForPlasticResponse = false;
          bottleMounted = false;
          loadCellState = LC_IDLE;
        } else {
          // No water detected, send scan command to Pi
          delay(2000); 
          Serial.println("PL");


          waitingForPlasticResponse = true;
          plasticResponseStartTime = millis();
        }
      }
      break;
      
    case LC_READINGS_COMPLETE:
      // Wait for response from Pi (handled in main loop)
      break;
  }
}

void tareScaleWhenGateClosed() {
  Serial.println("taring");
  delay(500); // Wait a bit for stability
  scale.tare(); // Tare EMPTY scale

}

// Function to move plastic sorter servo
void movePlasticSorter(int targetAngle) {
    plasticSorterTargetAngle = targetAngle;
    plasticSorterMoving = true;
    plasticSortServo.write(targetAngle);
    plasticSorterHoldStartTime = millis();
    plasticSorterHolding = true;
    
    Serial.print("Plastic sorter moving to: ");
    Serial.println(targetAngle);
}

void returnPlasticSorterToCenter() {
    plasticSorterTargetAngle = 90;
    plasticSorterMoving = true;
    plasticSortServo.write(sorterangle);
    plasticSorterMoving = false;
    plasticSorterHolding = false;
    Serial.println("Plastic sorter returned to center");
}

void setup() {
    Serial.begin(115200);   
    Serial2.begin(115200);  

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    pinMode(PAPER_RELAY_PIN, OUTPUT);
    digitalWrite(PAPER_RELAY_PIN, HIGH);
    
    pinMode(PLASTIC_RELAY_PIN, OUTPUT);
    digitalWrite(PLASTIC_RELAY_PIN, HIGH);
    
    pinMode(FAN_RELAY_PIN, OUTPUT);
    digitalWrite(FAN_RELAY_PIN, LOW);
    
    // Initialize shredder servo
    shredderServo.attach(SERVO_PIN);
    shredderServo.write(90);
    
    // Initialize redeem servos
    redeemServo1.attach(REDEEM_SERVO_PIN_1);
    redeemServo2.attach(REDEEM_SERVO_PIN_2);
    redeemServo3.attach(REDEEM_SERVO_PIN_3);
    redeemServo1.write(90);
    redeemServo2.write(90);
    redeemServo3.write(90);
    
    pinMode(PAPER_INSERT_IR_PIN, INPUT_PULLUP);
    pinMode(PAPER_POSITION_IR_PIN, INPUT_PULLUP); 
    paperRollerLeft.attach(PAPER_ROLLER_LEFT_PIN);
    paperRollerRight.attach(PAPER_ROLLER_RIGHT_PIN);
    stopPaperRollers();
    
    plasticGateServo.attach(PLASTIC_GATE_SERVO_PIN);
    plasticSortServo.attach(PLASTIC_SORT_SERVO_PIN);
    plasticGateServo.write(90);   // Closed position
    plasticSortServo.write(sorterangle);   // Center position
    
    // Initialize bottle mounting ultrasonic sensor
    pinMode(TRIG_BOTTLE_MOUNT, OUTPUT);
    pinMode(ECHO_BOTTLE_MOUNT, INPUT);
    
    // Initialize HX711 load cell (but don't tare yet)
    scale.begin(HX711_DAT_PIN, HX711_CLK_PIN);
    scale.set_scale(2280.f);      // calibration factor
    
    // Clear weight buffer
    for (int i = 0; i < WEIGHT_BUFFER_SIZE; i++) weightBuffer[i] = 0.0;
    
    // Initialize IR sensor
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
    
    // Initialize ultrasonic sensor pins
    pinMode(TRIG_PAPER, OUTPUT);
    pinMode(ECHO_PAPER, INPUT);
    pinMode(TRIG_PLASTIC, OUTPUT);
    pinMode(ECHO_PLASTIC, INPUT);
    pinMode(TRIG_SHREDDER, OUTPUT);
    pinMode(ECHO_SHREDDER, INPUT);
    
    // Set initial state for bin check
    lastBinCheck = millis();
    
    // Send initial OK status for all bins on boot
    delay(1000);
    Serial2.println("PAPER_BIN:OK");
    Serial2.println("PLASTIC_BIN:OK");
    Serial2.println("SHREDDER_BIN:OK");
    Serial.println("Initial bin status: OK for all bins");

    SPI.begin();            
    mfrc522.PCD_Init();     
    delay(100);
    
    Serial.println("=== BOTTLE MOUNT DETECTION STARTED ===");
    Serial.println("Distance Range: 25-35 cm");
    Serial.println("Stable Count Required: 5");
    Serial.println("================================");
}

void loop() {
    updateLoadCellStateMachine();

    // Check bins every "CHECK_INTERVAL" seconds
    if (millis() - lastBinCheck >= CHECK_INTERVAL) {
        checkBins();
        lastBinCheck = millis();
    }
    
    // Handle paper roller timeout (safety measure)
    if (paperRollingInProgress) {
        // Check if paper has reached position IR sensor during roll-in
        if (rollInDirection && digitalRead(PAPER_POSITION_IR_PIN) == LOW && !paperAtPosition) {
            paperAtPosition = true;
            paperPositionReached = true;
            
            // STOP THE ROLLERS WHEN PAPER REACHES SCANNING POSITION
            stopPaperRollers();
            
            // Send PA command for validation
            if (waitingForPaperResponse) {
                delay(2000); 
                Serial.println("PA");
            }
        }
        
        // Handle roller completion when duration is set (for roll-back/eject OR roll-in)
        if (paperRollDuration > 0 && (millis() - paperRollStartTime >= paperRollDuration)) {
            stopPaperRollers();
            paperRollDuration = 0;
            
            // RESET SYSTEM STATE AFTER ROLLERS HAVE FINISHED
            digitalWrite(PAPER_RELAY_PIN, HIGH);
            currentMaterial = "";
            paperAtPosition = false;
            paperPositionReached = false;
            waitingForPaperResponse = false;
            
            // This ensures shredding only starts AFTER paper is fully rolled in
            if (shreddcount >= 10) { // 10 VALID STRAIGHT TO TRIGGER AUTOMATIC SHREDDING 
                Serial2.println("=== AUTO SHREDDING ACTIVATED ===");
                
                // Turn off paper light if it's still on
                digitalWrite(PAPER_RELAY_PIN, HIGH);
                
                // Start shredding
                digitalWrite(RELAY_PIN, LOW);
                shredderServo.write(160);
                Serial2.println("STARTSHREDDING");
                
                // Shred for 50 seconds
                delay(50000);
                
                // Stop shredding
                digitalWrite(RELAY_PIN, HIGH);
                shredderServo.write(90);
                Serial2.println("STOPSHREDDING");
                
                shreddcount = 0; // Reset counter
                Serial.println("=== AUTO SHREDDING COMPLETED ===");
            }
        }
        
        // Safety timeout for roll-in (if paper doesn't reach IR2)
        if (rollInDirection && !paperAtPosition && paperRollDuration == 0 && 
            (millis() - paperRollStartTime >= rollonTimeout)) {
            stopPaperRollers();
            
            if (waitingForPaperResponse) {
                delay(2000);
                Serial.println("PA");
                paperResponseStartTime = millis();
            }
        }
        
        // Safety timeout for roll-back
        if (!rollInDirection && paperRollDuration == 0 && 
            (millis() - paperRollStartTime >= rollbackTimeout)) {
            Serial.println("Roll-back timeout - stopping rollers");
            stopPaperRollers();
            
            // Reset system after timeout
            digitalWrite(PAPER_RELAY_PIN, HIGH);
            currentMaterial = "";
            paperAtPosition = false;
            paperPositionReached = false;
            waitingForPaperResponse = false;
        }
    }
    
    // Handle paper response timeout (10 seconds)
    if (waitingForPaperResponse && (millis() - paperResponseStartTime >= PAPER_RESPONSE_TIMEOUT)) {
        Serial2.println("INVALID:PAPER");
        
        // Roll back (eject) paper on timeout
        startPaperRollers(false);
        paperRollStartTime = millis();
        paperRollDuration = 5000; // Roll back for 5 seconds
        
        waitingForPaperResponse = false;
        paperAtPosition = false;
        // currentMaterial will be reset after ejection completes
    }
    
    // Handle plastic bottle mounting detection
    if (waitingForPlasticInsert && plasticGateOpen) {
        bool bottleDetected = checkBottleMounted();
        
        // If bottle is mounted and 5 seconds have passed since detection
        if (bottleMountingDetected && 
            (millis() - bottleMountDetectedTime >= BOTTLE_MOUNT_CONFIRM_TIME) &&
            !bottleMounted) {
            
            bottleMounted = true;
            Serial.println("=== BOTTLE CONFIRMED MOUNTED - CLOSING GATE ===");
            
            // Close the plastic gate
            plasticGateServo.write(90);
            plasticGateOpen = false;
            
            // Reset bottle mounting detection for next cycle
            bottleMountingDetected = false;
            stableBottleCount = 0;
            
            // Start load cell state machine (NON-BLOCKING)
            loadCellState = LC_BOTTLE_MOUNTED;
            loadCellStateTime = millis();
        }
    }
    
    // Handle plastic gate timeout 
    if (plasticGateOpen && millis() - plasticGateOpenTime >= PLASTIC_GATE_TIMEOUT) {
        plasticGateServo.write(90); // Close gate
        plasticGateOpen = false;
        Serial.println("=== PLASTIC GATE TIMEOUT - NO BOTTLE DETECTED ===");
        
        // Reset bottle mounting detection
        bottleMountingDetected = false;
        stableBottleCount = 0;
        bottleMounted = false;
        
        // Reset load cell state
        loadCellState = LC_IDLE;
        
        // If we didn't detect water and didn't send scan yet
        if (waitingForPlasticInsert && loadCellState != LC_READINGS_COMPLETE) {
            delay(2000);
            Serial.println("PL");
            waitingForPlasticResponse = true;
            plasticResponseStartTime = millis();
        }
    }
    
    // Handle plastic response timeout 
    if (waitingForPlasticResponse && (millis() - plasticResponseStartTime >= plastictimeout)) {
        Serial.println("=== PLASTIC RESPONSE TIMEOUT - SENDING INVALID ===");
        Serial2.println("INVALID:PLASTIC");
        
        // Turn sorter left for 4 seconds
        movePlasticSorter(moveINVALID); // 170 = left
        waitingForPlasticResponse = false;
        waitingForPlasticInsert = false;
        currentMaterial = "";
        bottleMounted = false;
        loadCellState = LC_IDLE; // Reset load cell state
    }
    
    // Handle plastic sorter hold timeout (4 seconds)
    if (plasticSorterHolding && (millis() - plasticSorterHoldStartTime >= PLASTIC_SORTER_HOLD_TIME)) {
        returnPlasticSorterToCenter();
    }
    
    if (Serial2.available() > 0) {
        String command = Serial2.readStringUntil('\n');
        command.trim();
        Serial.println("Received from Pi: " + command);

        if (command == "PING") {
            Serial2.println("[Mega→Pi]: READY");
            checkBins();
        }
        
        // ==================== PAPER VALIDATION ====================
        else if (command == "CHECK:PAPER") {
            currentMaterial = "PAPER";
            waitingForPaperInsert = true;
            waitingForPaperResponse = false;
            paperAtPosition = false;
            paperPositionReached = false;
        }
        
        // ==================== PLASTIC VALIDATION ====================
        else if (command == "CHECK:PLASTIC") {
            digitalWrite(PLASTIC_RELAY_PIN, LOW);
            currentMaterial = "PLASTIC";
            waitingForPlasticInsert = true;
            waitingForPlasticResponse = false;
            waterDetected = false;
            bottleMounted = false;
            bottleMountingDetected = false;
            stableBottleCount = 0;
        
            // RESET AND TARE SCALE BEFORE OPENING GATE
            loadCellState = LC_IDLE;
            tareScaleWhenGateClosed();  // TARE WHILE GATE IS CLOSED!
        
            // Reset load cell state
            loadCellState = LC_WAITING_FOR_BOTTLE;
        
            // Open plastic gate
            plasticGateServo.write(180); 
            plasticGateOpen = true;
            plasticGateOpenTime = millis();
        
            Serial.println("=== PLASTIC DEPOSIT STARTED ===");
            Serial.println("Scale tared, gate opened - waiting for bottle...");
        }
        
        // ==================== REDEEM COMMANDS ====================
        else if (command == "REDEEM:SLOT1") {
            Serial2.println("REDEEM_TRYING");
            
            bool itemDropped = rotateAndCheckServo(redeemServo1, 90, 180, 10000);
            
            if (itemDropped) {
                Serial2.println("REDEEM_SUCCESS");
                Serial.println("REDEEM_SUCCESS:SLOT1");
            } else {
                Serial2.println("REDEEM_FAILED");
                Serial.println("REDEEM_FAILED: No item detected from slot 1");
            }
        }
        else if (command == "REDEEM:SLOT2") {
            Serial2.println("REDEEM_TRYING");
            
            bool itemDropped = rotateAndCheckServo(redeemServo2, 90, 180, 10000);
            
            if (itemDropped) {
                Serial2.println("REDEEM_SUCCESS");
                Serial.println("REDEEM_SUCCESS:SLOT2");
            } else {
                Serial2.println("REDEEM_FAILED");
                Serial.println("REDEEM_FAILED: No item detected from slot 2");
            }
        }
        else if (command == "REDEEM:SLOT3") {
            Serial.println("Starting redeem slot 3 rotation...");
            Serial2.println("REDEEM_TRYING");
            
            bool itemDropped = rotateAndCheckServo(redeemServo3, 90, 180, 10000);
            
            if (itemDropped) {
                Serial2.println("REDEEM_SUCCESS");
                Serial.println("REDEEM_SUCCESS:SLOT3");
            } else {
                Serial2.println("REDEEM_FAILED");
                Serial.println("REDEEM_FAILED: No item detected from slot 3");
            }
        }
        
        // ==================== SHREDDER CONTROL ====================
        else if (command == "START:SHREDDING") {
            digitalWrite(RELAY_PIN, LOW);
            shredderServo.write(160);
            Serial.println("Shredder STARTED");
            Serial2.println("SHREDDER:ON");
        }
        else if (command == "STOP:SHREDDING") {
            digitalWrite(RELAY_PIN, HIGH);
            shredderServo.write(90);
            Serial.println("Shredder STOPPED");
            Serial2.println("SHREDDER:OFF");
        }
        
        // ==================== RFID SCAN ====================
        else if (command == "RFID:SCAN") {
            Serial.println("RFID Scan Command Received");
            
            bool tagFound = false;
            unsigned long startTime = millis();
            unsigned long timeout = 30000;
            
            while (millis() - startTime < timeout) {
                if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
                    tagFound = true;
                    String tagID = "";
                    
                    for (byte i = 0; i < mfrc522.uid.size; i++) {
                        tagID += String(mfrc522.uid.uidByte[i], DEC);
                        if (i < mfrc522.uid.size - 1) {
                            tagID += ":";
                        }
                    }
                    
                    Serial2.println("RFID:" + tagID);
                    delay(500);
                    Serial2.println("RFID:" + tagID);
                    delay(1000);
                    Serial2.println("RFID:" + tagID);
                    mfrc522.PICC_HaltA();
                    break;
                }
                delay(100);
            }
            
            if (!tagFound) {
                Serial2.println("TIMEOUT: No RFID tag detected");
            }
        }
    }
    
    // ==================== PAPER INSERTION DETECTION ====================
    if (waitingForPaperInsert && digitalRead(PAPER_INSERT_IR_PIN) == LOW) {
        digitalWrite(PAPER_RELAY_PIN, LOW);
        Serial.println("Paper detected! Starting rollers...");
        waitingForPaperInsert = false;
        paperAtPosition = false;
        
        // Start rolling in - will stop when PAPER_POSITION_IR_PIN detects paper
        startPaperRollers(true);
        
        // Start waiting for response
        waitingForPaperResponse = true;
        paperResponseStartTime = millis();
    }
    
    // ==================== LISTEN TO PI FOR RESPONSES ====================
    if (Serial.available() > 0) {
        String megaResponse = Serial.readStringUntil('\n');
        megaResponse.trim();
        Serial.println("Received from Pi: " + megaResponse);
        
        if (currentMaterial != "") {
            if (currentMaterial == "PAPER" && waitingForPaperResponse) {
    if (megaResponse == "RP:VALID") {
    Serial2.println("VALID:PAPER");
    Serial.println("Sent to Pi: VALID:PAPER");
    
    // Roll paper INWARD (into shredder) for valid paper
    startPaperRollers(true); // true = roll IN/forward
    
    // Set a roll-in duration to feed into shredder
    paperRollStartTime = millis();
    paperRollDuration = 5000; // Roll in for 5 seconds into shredder
    
    paperPositionReached = false;
    waitingForPaperResponse = false; 
    shreddcount++;

}
    else if (megaResponse == "RP:INVALID" || megaResponse == "RP:UNCERTAIN") {
        Serial2.println("INVALID:PAPER");
        Serial.println("Sent to Pi: INVALID:PAPER");
        
        // Roll back (eject) for invalid paper - user gets it back
        startPaperRollers(false); // false = roll OUT/back
        paperRollStartTime = millis();
        paperRollDuration = 5000; // Roll back for 5 seconds to eject
        
        paperPositionReached = false;
         waitingForPaperResponse = false;
    }
}
            // PLASTIC RESPONSE HANDLING
            else if (currentMaterial == "PLASTIC" && waitingForPlasticResponse) {
                if (megaResponse == "RP:VALID") {
                    Serial2.println("VALID:PLASTIC");
                    Serial.println("Sent to Pi: VALID:PLASTIC");
                    
                    movePlasticSorter(moveVALID);
                    digitalWrite(PLASTIC_RELAY_PIN, HIGH);
                    currentMaterial = "";
                    waitingForPlasticInsert = false;
                    waitingForPlasticResponse = false;
                    bottleMounted = false;
                    loadCellState = LC_IDLE; // Reset load cell state
                }
                else if (megaResponse == "RP:INVALID" || megaResponse == "RP:UNCERTAIN") {
                    Serial2.println("INVALID:PLASTIC");
                    Serial.println("Sent to Pi: INVALID:PLASTIC");
                    
                    movePlasticSorter(moveINVALID);
                    digitalWrite(PLASTIC_RELAY_PIN, HIGH);
                    currentMaterial = "";
                    waitingForPlasticInsert = false;
                    waitingForPlasticResponse = false;
                    bottleMounted = false;
                    loadCellState = LC_IDLE; // Reset load cell state
                }
            }
        }
    }

   
    delay(10);
}  