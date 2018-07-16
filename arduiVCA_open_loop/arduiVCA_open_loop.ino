//
// Created by Hank on 7/16/2018.
//

#include <VCA_Plant.h>

VCA_Plant myVCA;

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Setup starts");

    myVCA.DriveMotorASin(0.5, 50, 10, 1000);

    myVCA.StopMotorA();

    Serial.println("setup ends");
}

void loop() {
    //Serial.println(myVCA.ReadMotorAPositionBit());

}