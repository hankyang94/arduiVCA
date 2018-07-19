//
// Created by Hank on 7/16/2018.
//

#include <VCA_Plant.h>
#include <PID_v1.h>

VCA_Plant myVCA;

void setup() {
    Serial.begin(9600);
    Serial1.begin(230400);
    while (!Serial);
    Serial.println("Setup starts");
    int start_time = millis();
    myVCA.DriveMotorASin(0.2, 20, 10, 1000);
    Serial.println(millis() - start_time);
    myVCA.StopMotorA();

    Serial.println("setup ends");
}

void loop() {
    //Serial.println(myVCA.ReadMotorAPositionBit());

}