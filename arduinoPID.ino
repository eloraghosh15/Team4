#include <AStar32U4Motors.h>
#include <Encoder.h>

// --- HARDWARE CONFIGURATION ---
AStar32U4Motors m; 

// --- MOTOR & ENCODER PINS ---
const int encoderRightPinA = 13;
const int encoderRightPinB = 16; 
const int encoderLeftPinA = 15; 
const int encoderLeftPinB = 14; 

Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

// --- ULTRASONIC & LINE SENSORS ---
#include <QTRSensors.h>
QTRSensors qtr;
const int TRIG_PIN = 11;
const int ECHO_PIN = 19; // A1

// --- PID VARIABLES ---
long posLeftCount = 0, posRightCount = 0;
long posLeftCountLast = 0, posRightCountLast = 0;

// Target Velocity in COUNTS PER INTERVAL (5ms)
double targetTicksPerLoopL = 0;
double targetTicksPerLoopR = 0;

// PID Errors
double cumErrorL = 0, cumErrorR = 0;
double lastErrorL = 0, lastErrorR = 0;

// PID Constants (Conservative Start)
double Kp = 1.5;
double Ki = 0.5;
double Kd = 0.0;

// Motor Commands
int cmdL = 0;
int cmdR = 0;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastPingTime = 0;
const int LOOP_INTERVAL = 5; // 5ms

// Serial
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  m.setM1Speed(0);
  m.setM2Speed(0);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, 8);
  qtr.setEmitterPins(4, 5); 

  Serial.println("<READY>");
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChar, receivedChars);
      parseData(); 
      newData = false;
  }

  // Poll Encoders Fast
  long currRight = encoderRight.read();
  long currLeft = encoderLeft.read();

  // Control Loop (200Hz)
  if (millis() - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = millis();
    
    posRightCount = currRight;
    posLeftCount = currLeft;
    
    // 1. Calculate Actual Speed (Ticks this loop)
    double speedL = posLeftCount - posLeftCountLast;
    double speedR = posRightCount - posRightCountLast;
    
    posLeftCountLast = posLeftCount;
    posRightCountLast = posRightCount;

    // 2. PID Calculation
    if (targetTicksPerLoopL == 0 && targetTicksPerLoopR == 0) {
        cmdL = 0; cmdR = 0;
        cumErrorL = 0; cumErrorR = 0;
    } else {
        cmdL = computePID(targetTicksPerLoopL, speedL, &cumErrorL, &lastErrorL);
        cmdR = computePID(targetTicksPerLoopR, speedR, &cumErrorR, &lastErrorR);
    }

    // 3. Drive Motors
    m.setM1Speed(cmdL);
    m.setM2Speed(cmdR);
    
    // 4. Report Data (Every 50ms)
    if (millis() - lastPingTime > 50) {
        lastPingTime = millis();
        reportData();
    }
  }
}

int computePID(double target, double current, double *cumError, double *lastError) {
    double error = target - current;
    *cumError += error;
    
    // Clamp Integral
    if (*cumError > 100) *cumError = 100;
    if (*cumError < -100) *cumError = -100;
    
    double dError = error - *lastError;
    *lastError = error;
    
    double output = (Kp * error) + (Ki * *cumError) + (Kd * dError);
    
    // FEED FORWARD (Critical for movement!)
    // If target is 5 ticks/loop, we guess we need ~50 power.
    // Adjust this multiplier (10.0) until it moves well.
    double feedForward = target * 10.0; 
    
    output += feedForward;
    
    return constrain((int)output, -400, 400);
}

void reportData() {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 10000); 
    float dist = duration * 0.0343 / 2.0;
    
    uint16_t sensors[8];
    qtr.read(sensors);
    int line = 0;
    for(int i=2; i<6; i++) if(sensors[i] > 600) line = 1;

    Serial.print("D:"); 
    Serial.print(dist); 
    Serial.print(","); 
    Serial.print(posLeftCount); 
    Serial.print(","); 
    Serial.print(posRightCount);
    Serial.print(",");
    Serial.println(line);
}

void parseData() {
    char *strtokIndx;
    strtokIndx = strtok(tempChar, ",");
    char commandChar = strtokIndx[0];
    
    if (commandChar == 'M') { // <M, 200, 200> (Raw Power Input converted to Target Ticks)
        // We convert Raw Power (0-400) to Target Ticks Per Loop (approx 0-15)
        // This lets Python still use 150/200 numbers, but Arduino runs PID.
        strtokIndx = strtok(NULL, ",");
        double rawL = atof(strtokIndx);
        targetTicksPerLoopL = rawL / 20.0; // Scaling factor
        
        strtokIndx = strtok(NULL, ",");
        double rawR = atof(strtokIndx);
        targetTicksPerLoopR = rawR / 20.0;
    }
    else if (commandChar == 'R') {
        encoderLeft.write(0); encoderRight.write(0);
        posLeftCount = 0; posRightCount = 0;
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
            } else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) recvInProgress = true;
    }
}