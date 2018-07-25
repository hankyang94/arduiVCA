//
// Created by Hank on 7/24/2018.
// used for hall effect sensor calibration
//

#define HallSerial Serial1

int idx;
int numVals;
int sensorSum;
int sensorVal;
int sensorAverage;
unsigned long startTime, endTime;
const int AD620OUT = 34;   // change this for motor A and B
void setup() {
  Serial.begin(9600);
  HallSerial.begin(230400);
  analogReadResolution(16);
  numVals = 2000;
  sensorSum = 0;
  sensorAverage = 0;
  sensorVal = 0;
  idx = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
//  startTime = micros();
  if (idx < numVals) {
      sensorVal = analogRead(AD620OUT);
      sensorSum += sensorVal;
      idx++;
  }
  else {
      sensorAverage = (int)((double)sensorSum / double(numVals));
      Serial.println(sensorAverage);
      sensorSum = 0;
      idx = 0;
  }
//  Serial.println(sensorVal);
//  Serial.print('\n');
  HallSerial.println(sensorVal);
//  endTime = micros();
//  Serial.print(endTime-startTime);
//  Serial.print('\n');
}
