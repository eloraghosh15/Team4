#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <QTRSensors.h>

// --- HARDWARE CONFIGURATION ---
AStar32U4Motors m; 
QTRSensors qtr;

// --- MOTOR & ENCODER PINS ---
const int encoderRightPinA = 13;
const int encoderRightPinB = 16;
const int encoderLeftPinA = 15; 
const int encoderLeftPinB = 14; 

Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

// --- ULTRASONIC PINS ---
const int TRIG_PIN = 11;
const int ECHO_PIN = 19;

// --- LINE SENSOR CONFIG ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition = 0;
int isCross = 0;

// --- VARIABLES ---
int leftMotorCmd = 0;
int rightMotorCmd = 0;
unsigned long lastPingTime = 0;

// --- STATE MACHINE ---
// 0: Idle, 1: Ultrasonic/Approach (<S>), 2: Line Follow (<L>)
int dataMode = 0;

// --- SERIAL PARSING ---
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  m.setM1Speed(0);
  m.setM2Speed(0);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Line Sensors (Pins from your original file)
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);
  qtr.setEmitterPins(4, 5); 

  // Calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("<READY>");
}

void loop() {
  // 1. READ SERIAL COMMANDS
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChar, receivedChars);
      parseData(); 
      newData = false;
  }

  // 2. POLL ENCODERS
  long currRight = encoderRight.read();
  long currLeft = encoderLeft.read();

  // 3. SENSOR REPORTING (Every 40ms)
  if (millis() - lastPingTime > 40) {
      lastPingTime = millis();

      if (dataMode == 1) { // ULTRASONIC / APPROACH MODE (<S>)
          digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
          digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
          digitalWrite(TRIG_PIN, LOW);
          long duration = pulseIn(ECHO_PIN, HIGH, 10000); 
          float dist = duration * 0.0343 / 2.0;

          // --- NEW: ROBUST LINE CHECK ---
          qtr.readLineBlack(sensorValues); // Read sensors
          
          bool lineFound = false;
          // Check ALL sensors. If ANY is dark (> 200), we hit the line.
          for (int i = 0; i < 8; i++) {
             if (sensorValues[i] > 200) {
               lineFound = true;
               break;
             }
          }
          // ------------------------------

          // Report D:Dist,LeftEnc,RightEnc,LineFound
          Serial.print("D:"); 
          Serial.print(dist); 
          Serial.print(","); 
          Serial.print(currLeft); 
          Serial.print(","); 
          Serial.print(currRight);
          Serial.print(",");        
          Serial.println(lineFound); // Sends 1 or 0
      }
      else if (dataMode == 2) { // LINE FOLLOW MODE (<L>)
          linePosition = qtr.readLineBlack(sensorValues);
          
          // Apply Repairs
          for (int i=0; i<8; i++) if (sensorValues[i] < 300) sensorValues[i] = 0;
          
          bool allZero = true;
          for(int i=0; i<8; i++) if(sensorValues[i] > 0) allZero = false;
          if(allZero) linePosition = 0;

          if (sensorValues[0] > 0 && sensorValues[1] == 0) linePosition = 1000;
          if (sensorValues[7] > 0 && sensorValues[6] == 0) linePosition = 5000;

          if (linePosition > 5000) linePosition = 5000;
          if (linePosition < 1000 && linePosition > 0) linePosition = 1000;

          if ((sensorValues[7] > 500) && (sensorValues[0] > 500)) isCross = 1;
          else isCross = 0;

          // Report L:LinePos,IsCross
          Serial.print("L:"); 
          Serial.print(linePosition); 
          Serial.print(","); 
          Serial.println(isCross);
      }
  }

  // 4. DRIVE MOTORS
  m.setM1Speed(leftMotorCmd);
  m.setM2Speed(rightMotorCmd);
}

// --- PARSE COMMANDS ---
void parseData() {
    char *strtokIndx;
    strtokIndx = strtok(tempChar, ",");
    char commandChar = strtokIndx[0];

    if (commandChar == 'M') { 
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) leftMotorCmd = atoi(strtokIndx);
        
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) rightMotorCmd = atoi(strtokIndx);
    }
    else if (commandChar == 'R') { 
        encoderLeft.write(0);
        encoderRight.write(0);
    }
    else if (commandChar == 'S') {
        dataMode = 1; // Switch to Ultrasonic Mode
    }
    else if (commandChar == 'L') {
        dataMode = 2; // Switch to Line Mode
        leftMotorCmd = 0; rightMotorCmd = 0; 
    }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) ndx = numChars - 1;
            }
            else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) recvInProgress = true;
    }
}