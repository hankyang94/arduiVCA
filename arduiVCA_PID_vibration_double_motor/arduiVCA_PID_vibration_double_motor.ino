//
// Created by Hank on 7/24/2018.
//

#include <VCA_Plant.h>
#include <PID_v1.h>
#include <math.h>

// define our VCA plant
VCA_Plant myVCA;

// define our PID controller
int loop_period = 1000;    // loop period in microseconds, 1kHz --> 1000us
double Setpoint_A, Input_A, Output_A;
double Kp_A = 0.2, Ki_A = 0, Kd_A = 0;   // for motor A, 0.25 is the max Kp
PID myPID_A(&Input_A, &Output_A, &Setpoint_A, Kp_A, Ki_A, Kd_A,  // proportional on error
          DIRECT);   // because the voltage decreases as position increases, here we use reverse
// if directly measure shaft position based on calibration, use DIRECT

double Setpoint_B, Input_B, Output_B;
double Kp_B = 0.3, Ki_B = 0, Kd_B = 0;   // for motor B, 0.3 is the max Kp
PID myPID_B(&Input_B, &Output_B, &Setpoint_B, Kp_B, Ki_B, Kd_B,  // proportional on error
            DIRECT);


// define vibration frequency
int vibFreq_A = 50;
double vibAmp_A = 1.0;
double offSet_A = 2;

int vibFreq_B = 50;
double vibAmp_B = 1.0;
double offSet_B = 2;

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
    myPID_A.SetMode(AUTOMATIC);   // automatic, not manual
    myPID_A.SetOutputLimits(-1.0, 1.0);  // output is duty cycle, so clamped between -1 and 1
    myPID_A.SetSampleTime(loop_period);    // Sample time in microseconds, 1kHz-->1000us

    myPID_B.SetMode(AUTOMATIC);   // automatic, not manual
    myPID_B.SetOutputLimits(-1.0, 1.0);  // output is duty cycle, so clamped between -1 and 1
    myPID_B.SetSampleTime(loop_period);    // Sample time in microseconds, 1kHz-->1000us

    start_time = micros();
}

void loop() {
    timer = micros();
    Setpoint_A = vibAmp_A * sin(2.0 * M_PI * vibFreq_A * (((double) (timer - start_time)) / 1e6)) + offSet_A;
    Setpoint_B = vibAmp_B * sin(2.0 * M_PI * vibFreq_B * (((double) (timer - start_time)) / 1e6)) + offSet_B;
//    Serial.println(Setpoint);
    Input_A = myVCA.ReadMotorAPositionMM();  // directly read position, use direct PID
    Input_B = myVCA.ReadMotorBPositionMM();
    myPID_A.ComputeWithoutTiming();
    myPID_B.ComputeWithoutTiming();
    myVCA.DriveMotorADuty(Output_A);
    myVCA.DriveMotorBDuty(Output_B);
    while (micros() - timer < loop_period);
}