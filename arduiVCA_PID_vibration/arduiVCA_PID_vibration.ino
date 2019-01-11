/*
// Created by Hank on 7/24/2018.
 This script controls the VCA at a fixed point.
*/

#include <VCA_Plant.h>
#include <PID_v1.h>
#include <math.h>

// define our VCA plant
VCA_Plant myVCA;

// define our PID controller
int loop_period = 1000;    // loop period in microseconds, 1kHz --> 1000us
double Setpoint, Input, Output;
double Kp = 0.2, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,  // proportional on error
          DIRECT);   // because the voltage decreases as position increases, here we use reverse
// if directly measure shaft position based on calibration, use DIRECT

// define vibration frequency
int vibFreq = 30;
double vibAmp = 1.0;
double offSet = 2.5;

int timer;
int start_time;

void setup() {
    Serial.begin(9600);
    Serial1.begin(230400);
    while (!Serial);
    Serial.println("Setup starts");
    Serial.println(M_PI);
    myVCA.StopMotor();
    Serial.println("setup ends");
    myPID.SetMode(AUTOMATIC);   // automatic, not manual
    myPID.SetOutputLimits(-1.0, 1.0);  // output is duty cycle, so clamped between -1 and 1
    myPID.SetSampleTime(loop_period);    // Sample time in microseconds, 1kHz-->1000us

    start_time = micros();
}

void loop() {
    timer = micros();
    Setpoint = vibAmp * sin(2.0 * M_PI * vibFreq * (((double) (timer - start_time)) / 1e6)) + offSet;
//    Serial.println(Setpoint);
    Input = myVCA.ReadMotorAPositionMM();  // directly read position, use direct PID
    myPID.ComputeWithoutTiming();
    myVCA.DriveMotorADuty(Output);
    while (micros() - timer < loop_period);
}