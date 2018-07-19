//
// Created by Hank on 7/16/2018.
//

#include <VCA_Plant.h>
#include <PID_v1.h>

// define our VCA plant
VCA_Plant myVCA;

// define our PID controller
int loop_period = 1000;    // loop period in microseconds, 1kHz --> 1000us
double Setpoint, Input, Output;
double Kp = 0.2, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,  // proportional on error
          REVERSE);   // because the voltage decreases as position increases, here we use reverse

int timer = micros();

void setup() {
    Serial.begin(9600);
    Serial1.begin(230400);
    while (!Serial);
    Serial.println("Setup starts");
    myVCA.StopMotorA();
    Serial.println("setup ends");
    Setpoint = 3.0;    // just do a fixed position control
    myPID.SetMode(AUTOMATIC);   // automatic, not manual
    myPID.SetOutputLimits(-1.0, 1.0);  // output is duty cycle, so clamped between -1 and 1
    myPID.SetSampleTime(loop_period);    // Sample time in microseconds, 1kHz-->1000us
}

void loop() {
    timer = micros();
    Input = myVCA.ReadMotorAPositionVoltage();
    myPID.ComputeWithoutTiming();
    myVCA.DriveMotorADuty(Output);
    while (micros() - timer < loop_period);
}