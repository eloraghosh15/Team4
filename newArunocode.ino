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

// --- ROBOT CONSTANTS ---
#define PI 3.141592653589
const int encoderResolution = 1440; 
const double d = 2.7559055; 

// --- VARIABLES ---
int leftMotorCmd = 0;
int rightMotorCmd = 0;
double leftMotorMax = 400; 
double rightMotorMax = 400;

long posLeftCount = 0;
long posRightCount = 0;
long posLeftCountLast = 0;
long posRightCountLast = 0;

double velLeft = 0;  
double velRight = 0; 

// --- PID CONTROL ---
const double interval = 5.0; 
unsigned long previousMillis = 0;
unsigned long priorTimeL, priorTimeR; 
double lastSpeedErrorL, lastSpeedErrorR; 
double cumErrorL, cumErrorR;
double maxErr = 20; 
double desVelL = 0.0; 
double desVelR = 0.0;

double kpL = 2.0, kiL = 0.65, kdL = 0.5; 
double kpR = 2.2, kiR = 0.65, kdR = 0.5;

// --- SERIAL PARSING ---
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
boolean newData = false;

// --- ULTRASONIC & LINE SENSORS ---
#include <QTRSensors.h>
QTRSensors qtr;
const int TRIG_PIN = 11;
const int ECHO_PIN = 19; 
unsigned long lastPingTime = 0;

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

  // Poll Encoders (Needed for Pin 13)
  long currRight = encoderRight.read();
  long currLeft = encoderLeft.read();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
     previousMillis = currentMillis;
     
     posRightCount = currRight;
     posLeftCount = currLeft;
     
     // 1. Calculate Velocity (For reporting, even if not using PID)
     long countInstantRight = posRightCount - posRightCountLast;
     long countInstantLeft = posLeftCount - posLeftCountLast;

     double rightRevSec = (countInstantRight * (1000.0 / interval)) / encoderResolution;
     double leftRevSec = (countInstantLeft * (1000.0 / interval)) / encoderResolution;

     velRight = rightRevSec * (d * PI);
     velLeft = leftRevSec * (d * PI);

     // 2. MOTOR CONTROL LOGIC
     // HYBRID MODE: If target is > 50, assume RAW PWM (-400 to 400).
     // If target is < 50, assume Velocity (Inches/Sec) and use PID.
     // This lets Python send "M,150,150" to force movement.
     
     if (abs(desVelL) > 50 || abs(desVelR) > 50) {
        // --- RAW POWER MODE ---
        leftMotorCmd = (int)desVelL;
        rightMotorCmd = (int)desVelR;
        
        // Reset PID terms so they don't wind up while in raw mode
        cumErrorL = 0; cumErrorR = 0;
     } 
     else {
        // --- PID VELOCITY MODE ---
        double newVelRight, newVelLeft;
        
        if (desVelR == 0 && desVelL == 0) {
            newVelRight = 0; newVelLeft = 0;
            cumErrorR = 0; cumErrorL = 0; 
        } else {
            newVelRight = drivePIDR(velRight);
            newVelLeft = drivePIDL(velLeft);
        }
        
        rightMotorCmd = constrain(newVelRight, -400, 400);
        leftMotorCmd = constrain(newVelLeft, -400, 400);
     }

     CommandMotors();
     
     posRightCountLast = posRightCount;
     posLeftCountLast = posLeftCount;
     
     // 3. REPORT DATA
     float dist = -1;
     if (millis() - lastPingTime > 60) {
        lastPingTime = millis();
        dist = readDistance();
     }
     
     if(dist != -1) {
        Serial.print("D:"); 
        Serial.print(dist); 
        Serial.print(","); 
        Serial.print(posLeftCount); 
        Serial.print(","); 
        Serial.println(posRightCount);
     }
  }
}

void CommandMotors(){  
  m.setM1Speed(leftMotorCmd);
  m.setM2Speed(rightMotorCmd);
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 15000); 
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

double drivePIDL(double curr) {
    unsigned long currentTime = millis();
    double elapsedTime = (double)(currentTime - priorTimeL);
    if(elapsedTime <= 0) elapsedTime = 1;

    double error = desVelL - curr;
    cumErrorL += error * elapsedTime;
    if(cumErrorL > maxErr) cumErrorL = maxErr;
    else if(cumErrorL < -maxErr) cumErrorL = -maxErr;

    double rateError = (error - lastSpeedErrorL)/elapsedTime;
    double out = kpL*error + kiL*cumErrorL + kdL*rateError;
    
    lastSpeedErrorL = error;
    priorTimeL = currentTime;
    return out; 
}

double drivePIDR(double curr) {
    unsigned long currentTime = millis();
    double elapsedTime = (double)(currentTime - priorTimeR);
    if(elapsedTime <= 0) elapsedTime = 1;

    double error = desVelR - curr;
    cumErrorR += error * elapsedTime;
    if(cumErrorR > maxErr) cumErrorR = maxErr;
    else if(cumErrorR < -maxErr) cumErrorR = -maxErr;

    double rateError = (error - lastSpeedErrorR)/elapsedTime;
    double out = kpR*error + kiR*cumErrorR + kdR*rateError;

    lastSpeedErrorR = error;
    priorTimeR = currentTime;
    return out; 
}

int motorVelToSpeedCommand(double Vel, double maxVel){
    int newSpeed = 0;
    return (int)Vel; 
}

void parseData() {
    char *strtokIndx;
    strtokIndx = strtok(tempChar, ",");
    char commandChar = strtokIndx[0];
    
    if (commandChar == 'M') { 
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) desVelL = atof(strtokIndx); 
        
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) desVelR = atof(strtokIndx); 
    }
    else if (commandChar == 'R') { 
        encoderLeft.write(0);
        encoderRight.write(0);
        posLeftCount = 0;
        posRightCount = 0;
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