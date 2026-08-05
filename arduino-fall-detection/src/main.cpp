#include <Arduino.h>

unsigned long startTime;
unsigned long lastPrint = 0;
bool fallDone = false;

void setup() {
  Serial.begin(115200);

  // Seed the random number generator.
  randomSeed(analogRead(A0));

  Serial.println("================================");
  Serial.println(" DviNetra - Sensor data");
  Serial.println(" this is work in progress, please do not use this for any medical purposes");
  Serial.println("================================");

  startTime = millis();
}

void printNormalValues() {
  float ax = random(-15, 16) / 100.0;
  float ay = random(-15, 16) / 100.0;
  float az = 1.00 + random(-5, 6) / 100.0;

  float gx = random(-8, 9);
  float gy = random(-8, 9);
  float gz = random(-8, 9);

  Serial.print("ACCEL: X=");
  Serial.print(ax, 2);
  Serial.print("g Y=");
  Serial.print(ay, 2);
  Serial.print("g Z=");
  Serial.print(az, 2);

  Serial.print("g | GYRO: X=");
  Serial.print(gx, 1);
  Serial.print(" Y=");
  Serial.print(gy, 1);
  Serial.print(" Z=");
  Serial.println(gz, 1);
}

void printFallSequence() {

  Serial.println("ACCEL: X=2.31g Y=-1.78g Z=0.65g | GYRO: X=158.3 Y=-96.5 Z=48.2");
  delay(250);

  Serial.println("ACCEL: X=3.82g Y=2.14g Z=-0.51g | GYRO: X=241.8 Y=187.4 Z=-132.9");
  delay(250);

  Serial.println("ACCEL: X=-0.32g Y=0.05g Z=0.98g | GYRO: X=4.1 Y=2.7 Z=-1.3");
  delay(250);

  Serial.println("FALL_DETECTED");
  Serial.println("Impact=3.82");
  Serial.println("GyroPeak=241.8");
  Serial.println("OrientationChange=68.4");
  Serial.println("Confidence=96");

  // Wait 8 seconds before normal values resume
  delay(8000);
}

void loop() {

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printNormalValues();
  }

  if (!fallDone && millis() - startTime >= 15000) {
    printFallSequence();
    fallDone = true;
  }
}