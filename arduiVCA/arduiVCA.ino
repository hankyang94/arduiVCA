/*
test simple VCA script
 This script reads ADC value and convert sensor position to mm, this script can be
 used for checking if calibration is done correctly.

 Run this script before running vibration so that motor does not go crazy.

*/

#include <VCA_Plant.h>

VCA_Plant myVCA;

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("in loop");
    int startTime = 0;
    int endTime = 0;
    for (int i = 0; i < 5; i++) {
        startTime = micros();
        Serial.println(myVCA.ReadMotorAPositionMM());
//        myVCA.DriveMotorADuty(0.5);
        endTime = micros();
        Serial.println(endTime - startTime);
        delayMicroseconds(450);
    }
    Serial.println(myVCA.ReadMotorAPositionBit());
    myVCA.StopMotor();

    for (int i = 0; i < 10; i++) {
        Serial.println(myVCA.ReadMotorAPositionBit());
    }
    
    Serial.println("setup ends");
}

void loop() {
    //Serial.println(myVCA.ReadMotorAPositionBit());
  
}
