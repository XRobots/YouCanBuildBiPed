void kinematics (int leg, float xIn, float yIn, float zIn, int interOn, int dur) {

    #define length1 85   // mm
    float angle1;
    float angle1a;
    float angle1b;
    float angle1c;
    float angle2;
    float angle2Degrees;
    float angle2aDegrees;
    float angle1Degrees;
    float z3;
    float shoulderAngle2;
    float shoulderAngle2Degrees;

    float shoulderAngleServo;
    float shoulderAngleServo2;
    float kneeAngleServo;
     
    // ** INTERPOLATION **
    // use Interpolated values if Interpolation is on
    if (interOn == 1) {
            
        if (leg == 1) {        // front right
            z = interpRZ.go(zIn,dur);
            x = interpRX.go(xIn,dur);
            y = interpRY.go(yIn,dur);
        }
      
        else if (leg == 2) {    // front left
            z = interpLZ.go(zIn,dur);
            x = interpLX.go(xIn,dur); 
            y = interpLY.go(yIn,dur);          
        }
    }

    else {
      x = xIn;
      z = zIn;
      y = yIn;
    }
      
    // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
    shoulderAngle2 = atan(x/z);     // calc how much extra to add to the shoulder joint    
    z3 = z/cos(shoulderAngle2);     // calc new leg length to feed to the next bit of code below

    // calculate leg length based on shin/thigh length and knee and shoulder angle
    angle1a = sq(length1) + sq(z3) - sq(length1);
    angle1b = 2 * length1 * z3;
    angle1c = angle1a / angle1b;
    angle1 = acos(angle1c);     // hip angle RADIANS
    angle2 = PI - (angle1 * 2); // knee angle RADIANS

    //calc degrees from angles
    angle2Degrees = angle2 * (180/PI);    // knee angle DEGREES
    angle2aDegrees = angle2Degrees / 2;    // half knee angle for each servo DEGREES
    shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // front/back shoulder offset DEGREES

    shoulderAngleServo = (angle2aDegrees-45)*7;        // remove defualt angle offset. Multiple degrees to get servo microseconds.
    kneeAngleServo = (angle2aDegrees-45)*7;
    shoulderAngleServo2 = shoulderAngle2Degrees*7;

    if(leg == 1) {            // right leg
        servo1Output = 1500 - shoulderAngleServo - shoulderAngleServo2;
        servo2Output = 1500 - kneeAngleServo + shoulderAngleServo2;
    }
    else if (leg == 2) {      // left leg
        servo3Output = 1500 + shoulderAngleServo + shoulderAngleServo2;
        servo4Output = 1500 + kneeAngleServo - shoulderAngleServo2;             
    }    

    y = constrain(y, -150, 50);   // stop knees knocking together when turning
    servo5Output = 1500 + y;      // add centre position and interpolated y value to rotation servo

    // write to servos
    servo1.writeMicroseconds(servo1Output + servo1Offset);
    servo2.writeMicroseconds(servo2Output + servo2Offset);
    servo3.writeMicroseconds(servo3Output + servo3Offset);
    servo4.writeMicroseconds(servo4Output + servo4Offset);
    servo5.writeMicroseconds(servo5Output + servo5Offset);    

}


