#include <ESP32Servo.h>     // ESP32 Servo library from libraries manager

// ramp lib
#include <Ramp.h>           // https://github.com/siteswapjuggler/RAMP

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

#define BNO08X_INT  19
#define BNO08X_RST  18
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

#include <esp_now.h>
#include <WiFi.h>

float pot1;           // receiving remote joystick
float pot2;           // receiving remote joystick
float pot1Filtered;       // receiving remote joystick filtered
float pot2Filtered;       // receiving remote joystick filtered
int but1;         // receiving button 1
int but2;         // receiving button 2
int robotMode;       // mode incremented or decremented by remote buttons
int modeDB1;     // mode debounce
int modeDB2;     // mode debounce

// IMU axis 

float roll;
float pitch;
float yaw;

float x = 0;
float y = 0;
float z = 140;

float xMain = 0;
float zMain = 140;
float yMain = 0;

float leg1z = 140;
float leg2z = 140;

int legx = 0;
float leg1x = 0;
float leg2x = 0;

// output for servos

float servo1Output = 1500;      // right upper leg
float servo2Output = 1500;      // right lower leg
float servo3Output = 1500;      // left upper leg
float servo4Output = 1500;      // left lower leg
float servo5Output = 1500;      // rotation axis

// offset for servos - start with zero and adjust until all servos are in the correct starting position with all legs down

int servo1Offset = -60;       // right upper leg - lower number maks the leg longer
int servo2Offset = 30;     // right lower leg - lower number maks the leg longer
int servo3Offset = 32;      // left upper leg - higher number makes the leg longer
int servo4Offset = 5;      // left lower leg - higher number makes the leg longer
int servo5Offset = 30;      // rotation axis - higher number closes legs more

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int a;    // pot1
  int b;    // pot2
  int c; // button 1
  int d; // button 2

} struct_message;

unsigned long currentMillis;
unsigned long previousMillis = 0;   // set up timers
long previousStepMillis = 0;        // set up timers
long previousLeg1Millis = 0;        // set up timers
long previousLeg2Millis = 0;        // set up timers

int stepTime = 500;
int leg1Time = 125;
int leg2Time = 125;

int leg1Trigger = 0;
int leg2Trigger = 0;

int stepCount = 0;
int leg1Count = 0;
int leg2Count = 0;
int walk1Action = 0;
int walk2Action = 0;

float long1Leg;
float long2Leg;
float shortLeg;

float legRot;
float leg1Rot;
float leg2Rot;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  pot2 = myData.a - 1919;     // modify offset to get zero when stick is stationary
  pot1 = myData.b - 1950;     // modify offset to get zero when stick is stationary
  but1 = myData.c;
  but2 = myData.d;

  // un-comment lines below to get centre stick positions
/*
  Serial.print(myData.a);
  Serial.print(" , ");
  Serial.print(myData.b);
  Serial.println(" , ");
*/

  // threshold value for control sticks
  // makes a dead-band of +/- 50
  if (pot1 > 50) {
    pot1 = pot1 -50;
  }
  else if (pot1 < -50) {
    pot1 = pot1 +50;
  }
  else {
    pot1 = 0;
  }    
  
  // threshold value for control sticks
  if (pot2 > 50) {
    pot2 = pot2 -50;
  }
  else if (pot2 < -50) {
    pot2 = pot2 +50;
  }
  else {
    pot2 = 0;
  }  
}

class Interpolation {
  public:
    rampFloat myRamp;
    int interpolationFlag = 0;
    float savedValue;

    float go(float input, int duration) {

      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value

      if (interpolationFlag == 0) {                                   // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }

      float output = myRamp.update();
      return output;
    }
};    // end of class

Interpolation interpRX;        // interpolation objects
Interpolation interpRZ;
Interpolation interpRY;

Interpolation interpLX;        // interpolation objects
Interpolation interpLZ;
Interpolation interpLY;

 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

   Wire.begin();

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
      Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
      while (1);
  }
  Serial.println("BNO08x found!");

  // attach servos to pins
  servo1.attach(13);        // right upper leg
  servo2.attach(12);        // left lower leg
  servo3.attach(14);        // left upper leg
  servo4.attach(27);        // right lower leg
  servo5.attach(26);        // rotation axis
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
    Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}
 
void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {     // this loop runs every 10ms
      previousMillis = currentMillis;

      pot1Filtered = filter(pot1, pot1Filtered, 6);      // filter sticks
      pot2Filtered = filter(pot2, pot2Filtered, 6);      // filter sticks

      // ** READ IMU DATA **
      if (myIMU.wasReset()) {
        Serial.println("sensor was reset ");
        setReports();
      }    
      // Has a new event come in on the Sensor Hub Bus?
      if (myIMU.getSensorEvent() == true) {    
        // is it the correct sensor data we want?
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {     
            pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
        }
      }
      // ** END OF READ IMU DATA **

      // ** processes remote buttons to increement and decrement mode/menu

      if (but1 == 0 && modeDB1 == 0) {
        modeDB1 = 1;
        robotMode = robotMode + 1;        
      }
      else if (but1 == 1) {
        modeDB1 = 0;
      }

      if (but2 == 0 && modeDB2 == 0) {
        modeDB2 = 1;
        robotMode = robotMode - 1;        
      }
      else if (but2 == 1) {
        modeDB2 = 0;
      }      
      robotMode = constrain(robotMode,0,3);   // constrain mode/menu options

      if (robotMode == 0) {
        // default positions at power on
        // write to servos
        servo1.writeMicroseconds(servo1Output + servo1Offset);
        servo2.writeMicroseconds(servo2Output + servo2Offset);
        servo3.writeMicroseconds(servo3Output + servo3Offset);
        servo4.writeMicroseconds(servo4Output + servo4Offset);
        servo5.writeMicroseconds(servo5Output + servo5Offset);
      }
      else if (robotMode == 1) {
        // inverse kinematics test       
        zMain = (pot1Filtered/60)+120;                       // map stick to robot height       
        xMain = pot2Filtered/60;                            // map stick to horizontal foot postion
        kinematics(1, xMain ,0, zMain ,0, 0);               // leg1 - send values to inverse kinematics 
        kinematics(2, xMain ,0, zMain ,0, 0);               // leg2 - send values to inverse kinematics 
      }
      else if (robotMode == 2) {
        // pose for walking
        kinematics(1, 0 , 0, 145 ,0, 0);                   // leg1 - send values to inverse kinematics 
        kinematics(2, 0 , 0 ,145 ,0, 0);                   // leg2 - send values to inverse kinematics
      }
      else if (robotMode == 3) {
        // dynamic walking      
      
        // *** START OF WALK ***

        long1Leg = 150;       // leg lengths for long and short leg legnths
        long2Leg = 150;
        shortLeg = 140;

        legRot = pot2/15;     // scale side-to-side joystick to get top rotation servo value
        legx = pot1/100;      // scale front-to-back stick for step length

        // dynamic step timing between steps - increases as it falls over further        
        stepTime = 240 + (abs(pitch) * 3);    
        stepTime = constrain(stepTime, 240, 260);
        
        // dynamic step timing of each step - decreases as it falls over further
        leg1Time = 150 - (abs(pitch)*1);    // dynamic step timing
        leg1Time = constrain(leg1Time, 130, 150);
        leg2Time = leg1Time;  

        // *** STATE MACHINES ***
               
        // *** alternate between legs ***
        if (stepCount == 0 && currentMillis - previousStepMillis >= stepTime) {
          stepCount = 1;
          leg1Trigger = 1;
          previousStepMillis = currentMillis;
        }
        else if (stepCount == 1 && currentMillis - previousStepMillis >= stepTime) {
          stepCount = 0;
          leg2Trigger = 1;
          previousStepMillis = currentMillis;
        }
        // *** end of alternate between legs ***

        // *** leg1 - right leg ***
        if (leg1Trigger == 1 && leg1Count == 0 && currentMillis - previousLeg1Millis >= leg1Time) {
          leg1z = shortLeg;
          previousLeg1Millis = currentMillis;
          leg1Trigger = 0;
          leg1Count = 1;
        }
        else if (leg1Count == 1 && currentMillis - previousLeg1Millis >= leg1Time) {
          leg1x = legx *-1;
          leg1Rot = legRot;
          previousLeg1Millis = currentMillis;
          leg1Count = 2;
        }
        else if (leg1Count == 2 && currentMillis - previousLeg1Millis >= leg1Time) {
          leg1z = long1Leg;          
          previousLeg1Millis = currentMillis;
          leg1Count = 3;
        }
        else if (leg1Count == 3 && currentMillis - previousLeg1Millis >= leg1Time) {
          leg1x = legx;
          leg1Rot = legRot * -1;
          previousLeg1Millis = currentMillis;
          leg1Count = 0;
        }

        // *** leg2 - left leg ***
        if (leg2Trigger == 1 && leg2Count == 0 && currentMillis - previousLeg2Millis >= leg2Time) {
          leg2z = shortLeg;
          previousLeg2Millis = currentMillis;
          leg2Trigger = 0;
          leg2Count = 1;
        }
        else if (leg2Count == 1 && currentMillis - previousLeg2Millis >= leg2Time) {
          leg2x = legx *-1;
          leg2Rot = legRot;
          previousLeg2Millis = currentMillis;
          leg2Count = 2;
        }
        else if (leg2Count == 2 && currentMillis - previousLeg2Millis >= leg2Time) {
          leg2z = long2Leg;          
          previousLeg2Millis = currentMillis;
          leg2Count = 3;  
        }
        else if (leg2Count == 3 && currentMillis - previousLeg2Millis >= leg2Time) {
          leg2x = legx;
          leg2Rot = legRot *-1;
          previousLeg2Millis = currentMillis;
          leg2Count = 0;
        }

        // *** END OF STATE MACHINES ***

        // *** send positions to inverse kinematics        
        kinematics(1, (leg1x + (legx)/4), leg1Rot ,leg1z ,1, leg1Time);                   // leg1 - send values to inverse kinematics 
        kinematics(2, (leg2x + (legx)/4), leg2Rot ,leg2z ,1, leg2Time);                   // leg2 - send values to inverse kinematics
      }      

    }  // end of 10ms timed loop
    
} // end of main loop


// motion filter to filter motions
float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}
