/*
test simple VCA script

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
        Serial.println(myVCA.ReadMotorBPositionMM());
//        myVCA.DriveMotorADuty(0.5);
        endTime = micros();
        Serial.println(endTime - startTime);
        delayMicroseconds(450);
    }
    Serial.println(myVCA.ReadMotorBPositionBit());
    myVCA.StopMotor();

    for (int i = 0; i < 10; i++) {
        Serial.println(myVCA.ReadMotorBPositionBit());
    }
    
    Serial.println("setup ends");
}

void loop() {
    //Serial.println(myVCA.ReadMotorAPositionBit());
  
}
