#include <QTRSensors.h>
#include <AStar32U4Motors.h>
AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing


//motor command variables
int leftMotor=0; //int leftMotor
int rightMotor=0;
int isCross=0;

  
boolean newData = false;





//=====================================================

void setup() {
   pinMode(3, OUTPUT); //left motor
   pinMode(2,OUTPUT); //left motor
    Serial.begin(115200);
    qtr.setTypeRC(); //this allows us to read the line sensor from didgital pins

    //arduino pin sensornames I am using: 7, 18, 23 aka A5. note:PIN A1 DID NOT WORK WITH ANY SENSOR!!, 20, 21, 22, 8, 6. UNHOOK THE BLUE JUMPER LABELED BUZZER ON THE ASTAR or pin 6 will cause the buzzer to activate.
//    qtr.setEmitterPins(4, 5);
//     QTRReadMode::On;
    qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);

    calibrateSensors();
    qtr.setEmitterPins(4, 5); //can get away with a single emitter pin providing power to both emitters
     QTRReadMode::On; //emitters on measures active reflectance instead of ambient light levels, better becasue the ambient light level will change as the robot moves around the board but the reflectance levels will not
    Serial.println("<Arduino is ready>");
}

//====================================================

void loop() {

//////////////////////////////////////////REPAIRING THE READLINEBLACK FUNCTION SO THE LINEPOSITION VARIABLE IS ACTUALLY USEFULL FOR STUDENTS//////////////////////////////////////////////

linePosition = qtr.readLineBlack(sensorValues);

//REPAIR NUMBER ONE:  every sensor reading under 300 is a noisy and messes up the lineposition measurment, so this for loop filters it out
for (int i=0; i<= 7; i++){
   if (sensorValues[i] <300){
       sensorValues[i]=0;     
    }
}


// REPAIR NUMBER 2: checking if all my sensorvalues are zero and then setting lineposition to zero,sometimes if all sensorvalues are zero lineposition will be set to 7k and this fixes that
if (sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
  linePosition=0;
}

//REPAIR NUMBER THREE: if only sensor 0 or sensor 8 are reading measurments, then the low level function that calculates linePosition will set it to values that dare not representative of where the sensor array
//is actually located relative to the line, the below loops fix that by setting linePosition to  1000 if ONLY sensor 0 sees anything, and setting linePosition to 5000 if ONLY sensor 7 sees anything
if (sensorValues[0] >0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
  linePosition=1000;
} 
if (sensorValues[7] >0 && sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0){
  linePosition=5000;
}

//REPAIR NUMBER 4: there are still situations where linePosition is somehow greater than 5000 or 0<linePosition<1000, so I am hard capping linePosition to be between 1000 and 5000 when linePosition is greater
//than zero.
if (linePosition > 5000){
  linePosition = 5000;
}
if (linePosition < 1000 && linePosition > 0){
  linePosition = 1000;
}

//this loop uses the leftmost and rightmost sensors to determin if the robot is at a cross. If both of those sensors read high, then the robot is at a cross. 
if ((sensorValues[7] > 500) && (sensorValues[0] > 500)){
  isCross = 1;
}else{
    isCross = 0;
  }

//now linePosition returns a value between 1000 and 5000 that is proportonal to the sensor's position relative to the line, and the value isCross is set to 1 when both sensor 0 and 7 are over a line


//////////////////////////////////////////////////////////////////////////////////////////////////////////END OF REPAIRS TO LINEPOSITION///////////////////////////////////////////////////////////////////////////

  

    recvWithStartEndMarkers(); //this function is in charge of taking a peice of data that looks like <17,16> 
                               //turning it into a string looking like 17,16 and then setting newdata to true,
                               //letting the rest of the program know a packet of data is ready to be analyzed, does all this without blocking
    if (newData == true) { //newData will be true when recvWithStartEndMarkers(); has finished recieving a whole set of data from Rpi (a set of data is denoted as being containted between <>)
      
      strcpy(tempChar, receivedChars); //this line makes a copy of recievedChars for parsing in parseData, I do this becasue strtok() will alter any string I give it,I want to preserve the origonal data
      parseData(); //right now parseData only parses a string of 2 numbers seperated by commas into floats
                   //so the string 17.5,16 becomes two floats; 17.5 and 16


      //below this comment and between setting newData to false is where you want to send the Rpi whatever data you want.
      Serial.print(linePosition);
      Serial.print(",");
      Serial.print(isCross);
      Serial.print(",");
      Serial.print(leftMotor);
      Serial.print(",");
      Serial.println(rightMotor);
      
      newData = false;

      
      //sendDataToRpi(); //unused
                   
    }



    commandMotors(); //we want this to happen outside of our newdata=true loop so it is never blocked
}


//======================================================


void parseData(){
  


  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand

  
  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  leftMotor = atoi(strtokIndexer); //converts strtokIndexer into a int
  

  strtokIndexer= strtok(NULL, ","); //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why 
  rightMotor = atoi(strtokIndexer);

  
  //now that we have extracted the data from the Rpi as floats, we can use them to command actuators somewhere else in the code
  
}

//==========================================


//=======================================

void commandMotors(){
  m.setM1Speed(leftMotor);
  m.setM2Speed(rightMotor);
}


//=========================================================


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
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need this
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//==================================================================

void calibrateSensors(){

  //THE SENSORS ONLY CALIBRATE WHEN YOU UPLOAD NEW ARDUINO CODE TO THE ASTAR. after that the sensors STAY calibrated as long as the Astar has power.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
                                   ///while calibrating, move the sensor over the line a couple times

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10s seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
}
