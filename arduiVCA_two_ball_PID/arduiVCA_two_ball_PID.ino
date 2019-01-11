//
// Created by Hank on 7/24/2018.
// this is the script for running experiments on phantoms
// You have to open the serial monitor to make motors vibrate!
//

#include <VCA_Plant.h>
#include <PID_v1.h>
#include <math.h>

// define ARF signal pin
const int ARFSignalPin = 24;   // digital read ARF signal pin 24, sent from another arduino that is dedicated to
                               // do signal processing. Later this pin will come from the same Teensy board that is doing
                               // motion control


// define our VCA plant
VCA_Plant myVCA;

// define our PID controller
int loop_period = 1000;    // loop period in microseconds, 1kHz --> 1000us
double Setpoint_A, Input_A, Output_A;
double Kp_A = 0.2, Ki_A = 1e-4, Kd_A = 1e-5;   // for motor A, transient PID for one-cycle vibration
                                                // for motor A, 0.2, 0, 0 would be good for continuous vibration
PID myPID_A(&Input_A, &Output_A, &Setpoint_A, Kp_A, Ki_A, Kd_A,  // proportional on error
          DIRECT);   // because the voltage decreases as position increases, here we use reverse
// if directly measure shaft position based on calibration, use DIRECT

double Setpoint_B, Input_B, Output_B;
double Kp_B = 0.17, Ki_B = 1e-9, Kd_B = 1e-5;   // for motor B, 0.3 is the max Kp
                                                // for motor B, 0.2, 0, 0, would be good for continuous vibration
PID myPID_B(&Input_B, &Output_B, &Setpoint_B, Kp_B, Ki_B, Kd_B,  // proportional on error
            DIRECT);


// define vibration frequency
int vibFreq_A = 50;
double vibAmp_A = 2;
double offSet_A = 3.0;

int vibFreq_B = 50;
double vibAmp_B = 2;
double offSet_B = 3.0;

const int num_cycles = 2;
const int cycle_gap_time = 2000;   // gap time between consecutive cycles
const int slack_time = 10;   // slack_time is used to make the vibration complete
int vib_duration;   // total vibration time, include gap
int vib_cycle;      // vibration time for each cycle

int main_timer;
int isr_timer;
int start_time;

void setup() {
    Serial.begin(9600);
    Serial1.begin(500000);
//    Serial1.begin(230400);
    while (!Serial);
    Serial.println("Setup starts");
    myVCA.StopMotor();
    Serial.println("setup ends");
    myPID_A.SetMode(AUTOMATIC);   // automatic, not manual
    myPID_A.SetOutputLimits(-1.0, 1.0);  // output is duty cycle, so clamped between -1 and 1
    myPID_A.SetSampleTime(loop_period);    // Sample time in microseconds, 1kHz-->1000us

    myPID_B.SetMode(AUTOMATIC);   // automatic, not manual
    myPID_B.SetOutputLimits(-1.0, 1.0);  // output is duty cycle, so clamped between -1 and 1
    myPID_B.SetSampleTime(loop_period);    // Sample time in microseconds, 1kHz-->1000us

    vib_cycle = 1 * (int)(1.0/(double)vibFreq_A * 1e6);
    vib_duration = num_cycles * vib_cycle + (num_cycles-1) * cycle_gap_time;
    Serial.println(vib_duration);
    attachInterrupt(digitalPinToInterrupt(ARFSignalPin), vibrate, RISING);
//    pinMode(ARFSignalPin, INPUT);    // set ARF signal pin to be input, so that it can read voltage HIGH and LOW
}

void loop() {

    main_timer = micros();
    Setpoint_A = offSet_A;
//    Serial.println(Setpoint_A);
    Setpoint_B = offSet_B;
    Input_A = myVCA.ReadMotorAPositionMM();  // directly read position, use direct PID
    Input_B = myVCA.ReadMotorBPositionMM();
    myPID_A.ComputeWithoutTiming();
    myPID_B.ComputeWithoutTiming();
    myVCA.DriveMotorADuty(Output_A);
    myVCA.DriveMotorBDuty(Output_B);
    while (micros() - main_timer < loop_period);

}

void vibrate() {
    cli();    // close interrupts
    Serial.println("vibrate");
    start_time = micros();
    for (int i = 0; i < num_cycles; i++) {
        while (micros() - start_time < ( (i+1) * vib_cycle + i * cycle_gap_time + slack_time)) {
            isr_timer = micros();
            Setpoint_A = - vibAmp_A * cos(2.0 * M_PI * vibFreq_A * (((double) (isr_timer - start_time)) / 1e6)) + vibAmp_A + offSet_A;
            Setpoint_B = - vibAmp_B * cos(2.0 * M_PI * vibFreq_B * (((double) (isr_timer - start_time)) / 1e6)) + vibAmp_B + offSet_B;
//        Serial.println(Setpoint_A);
//        Serial.println(Setpoint_B);
            Input_A = myVCA.ReadMotorAPositionMM();  // directly read position, use direct PID
            Input_B = myVCA.ReadMotorBPositionMM();
//        Serial.println(Input_A);
            myPID_A.ComputeWithoutTiming();
            myPID_B.ComputeWithoutTiming();
            myVCA.DriveMotorADuty(Output_A);
            myVCA.DriveMotorBDuty(Output_B);
            while (micros() - isr_timer < loop_period);
        }
        while (micros() - start_time < ( (i+1) * vib_cycle + (i+1)*cycle_gap_time + slack_time)) {
            isr_timer = micros();
            Setpoint_A = - vibAmp_A * cos(2.0 * M_PI * vibFreq_A * (((double) (isr_timer - start_time)) / 1e6)) + vibAmp_A + offSet_A;
            Setpoint_B = - vibAmp_B * cos(2.0 * M_PI * vibFreq_B * (((double) (isr_timer - start_time)) / 1e6)) + vibAmp_B + offSet_B;
            Input_A = myVCA.ReadMotorAPositionMM();  // directly read position, use direct PID
            Input_B = myVCA.ReadMotorBPositionMM();
//        Serial.println(Input_A);
            myPID_A.ComputeWithoutTiming();
            myPID_B.ComputeWithoutTiming();
            myVCA.DriveMotorADuty(Output_A);
            myVCA.DriveMotorBDuty(Output_B);
            while (micros() - isr_timer < loop_period);
        }
    }
    Serial.println(micros() - start_time);
    sei();   // enable interrupts
}
