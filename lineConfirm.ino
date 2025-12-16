#include <QTRSensors.h>
#include <AStar32U4Motors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(115200);
  
  // SETUP PINS
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);
  qtr.setEmitterPins(4, 5);

  // CALIBRATION LOOP
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("CALIBRATING... MOVE ROBOT OVER LINE!");
  
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration Done.");
  delay(1000);
}

void loop() {
  // Read sensors
  qtr.readLineBlack(sensorValues);

  // Print Raw Values
  Serial.print("Left[7]: ");
  Serial.print(sensorValues[7]);
  Serial.print("\t Right[0]: ");
  Serial.print(sensorValues[0]);
  
  // Print ALL values for sanity check
  Serial.print("\t | ALL: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  
  Serial.println();
  delay(100);
}