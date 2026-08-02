#include <Arduino.h>
#include <Wire.h>

/* MPU6050 setup */
const byte MPU = 0x68;
const byte ACCEL_XOUT_H = 0x3B;

const float ACCEL_SCALE = 4096.0f;  // MPU6050 accel set to +/-8g
const float GYRO_SCALE = 65.5f;     // MPU6050 gyro set to +/-500 dps

const unsigned long SAMPLE_TIME = 20;
const unsigned long PRINT_TIME = 1000;
const unsigned long PAUSE_TIME = 10000;

/* Ultra-low thresholds: high sensitivity, high false-alarm risk */
const float MOTION_ACCEL_TRIGGER = 1.02f;
const float MOTION_GYRO_TRIGGER = 12.0f;

const float IMPACT_LIMIT = 1.05f;
const float GYRO_PEAK_LIMIT = 18.0f;
const float ORIENTATION_LIMIT = 5.0f;

const float STILL_ACCEL_MIN = 0.40f;
const float STILL_ACCEL_MAX = 1.60f;
const float STILL_GYRO_LIMIT = 100.0f;

const float REFERENCE_ACCEL_MIN = 0.60f;
const float REFERENCE_ACCEL_MAX = 1.40f;
const float REFERENCE_GYRO_LIMIT = 55.0f;

const unsigned long EVENT_WINDOW = 2000;
const unsigned long STILL_CONFIRM_TIME = 200;
const unsigned long POST_IMPACT_TIMEOUT = 8000;

/* Missing in the previous sketch: required by updatePosture(). */
const float POSTURE_FILTER = 0.18f;

enum FallState {
  NORMAL,
  POSSIBLE_FALL_MOTION,
  IMPACT_DETECTED,
  POST_IMPACT_MONITORING
};

struct Reading {
  float ax, ay, az;
  float gx, gy, gz;
  float accel;
  float gyro;
};

FallState state = NORMAL;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float impactG = 0;
float gyroPeak = 0;
float orientationChange = 0;

/* Gravity direction before and after the possible fall. */
float normalGX = 0;
float normalGY = 0;
float normalGZ = 1;

float referenceGX = 0;
float referenceGY = 0;
float referenceGZ = 1;

float postGX = 0;
float postGY = 0;
float postGZ = 1;

bool haveNormalPosture = false;
bool havePostPosture = false;

unsigned long lastSample = 0;
unsigned long lastPrint = 0;
unsigned long stateSince = 0;
unsigned long stillSince = 0;
unsigned long pauseUntil = 0;

int16_t read16(byte high, byte low) {
  return (int16_t)(((uint16_t)high << 8) | low);
}

bool writeRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readSensor(Reading &r) {
  byte data[14];

  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU, (byte)14) != 14) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (byte i = 0; i < 14; i++) {
    data[i] = Wire.read();
  }

  r.ax = read16(data[0], data[1]) / ACCEL_SCALE;
  r.ay = read16(data[2], data[3]) / ACCEL_SCALE;
  r.az = read16(data[4], data[5]) / ACCEL_SCALE;

  r.gx = read16(data[8], data[9]) / GYRO_SCALE - gyroOffsetX;
  r.gy = read16(data[10], data[11]) / GYRO_SCALE - gyroOffsetY;
  r.gz = read16(data[12], data[13]) / GYRO_SCALE - gyroOffsetZ;

  r.accel = sqrt(r.ax * r.ax + r.ay * r.ay + r.az * r.az);
  r.gyro = sqrt(r.gx * r.gx + r.gy * r.gy + r.gz * r.gz);

  return true;
}

void calibrateGyro() {
  const byte samples = 120;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  byte count = 0;

  Serial.println(F("Keep MPU6050 still: calibrating gyro..."));

  for (byte i = 0; i < samples; i++) {
    Reading r;

    if (readSensor(r)) {
      sumX += r.gx;
      sumY += r.gy;
      sumZ += r.gz;
      count++;
    }

    delay(SAMPLE_TIME);
  }

  if (count > 0) {
    gyroOffsetX = sumX / count;
    gyroOffsetY = sumY / count;
    gyroOffsetZ = sumZ / count;
  }

  Serial.println(F("Gyro calibration complete."));
}

void normalizeVector(float &x, float &y, float &z) {
  float length = sqrt(x * x + y * y + z * z);

  if (length > 0.001f) {
    x /= length;
    y /= length;
    z /= length;
  }
}

/* Smooth the measured gravity direction after motion has settled. */
void updatePosture(const Reading &r, float &x, float &y, float &z) {
  if (r.accel < 0.1f) {
    return;
  }

  float nx = r.ax / r.accel;
  float ny = r.ay / r.accel;
  float nz = r.az / r.accel;

  x += POSTURE_FILTER * (nx - x);
  y += POSTURE_FILTER * (ny - y);
  z += POSTURE_FILTER * (nz - z);

  normalizeVector(x, y, z);
}

float calculateOrientationChange() {
  float dot = referenceGX * postGX +
              referenceGY * postGY +
              referenceGZ * postGZ;

  if (dot > 1.0f) dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;

  return acos(dot) * 57.29578f;
}

bool isStill(const Reading &r) {
  return r.accel >= STILL_ACCEL_MIN &&
         r.accel <= STILL_ACCEL_MAX &&
         r.gyro <= STILL_GYRO_LIMIT;
}

bool isReferenceSample(const Reading &r) {
  return r.accel >= REFERENCE_ACCEL_MIN &&
         r.accel <= REFERENCE_ACCEL_MAX &&
         r.gyro <= REFERENCE_GYRO_LIMIT;
}

void resetFallData() {
  impactG = 0;
  gyroPeak = 0;
  orientationChange = 0;
  stillSince = 0;
  havePostPosture = false;
}

int calculateConfidence(unsigned long stillDuration) {
  float impactScore = (impactG - IMPACT_LIMIT) / 2.8f;
  float gyroScore = (gyroPeak - GYRO_PEAK_LIMIT) / 200.0f;
  float orientationScore = (orientationChange - ORIENTATION_LIMIT) / 63.0f;
  float stillScore = ((float)stillDuration - STILL_CONFIRM_TIME) / 1600.0f;

  if (impactScore < 0) impactScore = 0;
  if (gyroScore < 0) gyroScore = 0;
  if (orientationScore < 0) orientationScore = 0;
  if (stillScore < 0) stillScore = 0;

  if (impactScore > 1) impactScore = 1;
  if (gyroScore > 1) gyroScore = 1;
  if (orientationScore > 1) orientationScore = 1;
  if (stillScore > 1) stillScore = 1;

  int confidence = 40 +
                   (int)(impactScore * 20.0f) +
                   (int)(gyroScore * 15.0f) +
                   (int)(orientationScore * 15.0f) +
                   (int)(stillScore * 10.0f);

  if (confidence > 100) confidence = 100;
  return confidence;
}

/* Python can continue to search for the first line: FALL_DETECTED */
void printFall(unsigned long stillDuration) {
  Serial.println(F("FALL_DETECTED"));

  Serial.print(F("Impact="));
  Serial.println(impactG, 2);

  Serial.print(F("GyroPeak="));
  Serial.println(gyroPeak, 1);

  Serial.print(F("OrientationChange="));
  Serial.println(orientationChange, 1);

  Serial.print(F("Confidence="));
  Serial.println(calculateConfidence(stillDuration));
}

void printReading(const Reading &r) {
  Serial.print(F("ACCEL: X="));
  Serial.print(r.ax, 2);
  Serial.print(F("g Y="));
  Serial.print(r.ay, 2);
  Serial.print(F("g Z="));
  Serial.print(r.az, 2);

  Serial.print(F("g | GYRO: X="));
  Serial.print(r.gx, 1);
  Serial.print(F(" Y="));
  Serial.print(r.gy, 1);
  Serial.print(F(" Z="));
  Serial.println(r.gz, 1);
}

void checkFall(const Reading &r, unsigned long now) {
  /* Capture an initial posture reference after boot. */
  if (!haveNormalPosture && r.accel > 0.1f) {
    normalGX = r.ax / r.accel;
    normalGY = r.ay / r.accel;
    normalGZ = r.az / r.accel;
    normalizeVector(normalGX, normalGY, normalGZ);
    haveNormalPosture = true;
  }

  /* Update the reference only while relatively calm. */
  if (state == NORMAL && isReferenceSample(r)) {
    updatePosture(r, normalGX, normalGY, normalGZ);
  }

  switch (state) {
    case NORMAL:
      /* No free-fall condition is required. */
      if (r.accel >= MOTION_ACCEL_TRIGGER ||
          r.gyro >= MOTION_GYRO_TRIGGER) {
        resetFallData();

        impactG = r.accel;
        gyroPeak = r.gyro;

        referenceGX = normalGX;
        referenceGY = normalGY;
        referenceGZ = normalGZ;

        stateSince = now;
        state = POSSIBLE_FALL_MOTION;
      }
      break;

    case POSSIBLE_FALL_MOTION:
      /* Store highest acceleration and angular velocity in the event. */
      if (r.accel > impactG) impactG = r.accel;
      if (r.gyro > gyroPeak) gyroPeak = r.gyro;

      if (impactG >= IMPACT_LIMIT &&
          gyroPeak >= GYRO_PEAK_LIMIT) {
        stateSince = now;
        state = IMPACT_DETECTED;
      } else if (now - stateSince > EVENT_WINDOW) {
        state = NORMAL;
      }
      break;

    case IMPACT_DETECTED:
      stillSince = 0;
      havePostPosture = false;
      stateSince = now;
      state = POST_IMPACT_MONITORING;
      break;

    case POST_IMPACT_MONITORING:
      if (r.accel > impactG) impactG = r.accel;
      if (r.gyro > gyroPeak) gyroPeak = r.gyro;

      if (isStill(r)) {
        if (stillSince == 0) {
          stillSince = now;

          postGX = r.ax / r.accel;
          postGY = r.ay / r.accel;
          postGZ = r.az / r.accel;
          normalizeVector(postGX, postGY, postGZ);

          havePostPosture = true;
        } else {
          updatePosture(r, postGX, postGY, postGZ);
        }

        orientationChange = calculateOrientationChange();

        /* Confirm changed posture plus post-impact stillness. */
        if (now - stillSince >= STILL_CONFIRM_TIME) {
          if (orientationChange >= ORIENTATION_LIMIT) {
            printFall(now - stillSince);
            pauseUntil = now + PAUSE_TIME;
          }

          state = NORMAL;
        }
      } else {
        stillSince = 0;
        havePostPosture = false;
      }

      if (now - stateSince > POST_IMPACT_TIMEOUT) {
        state = NORMAL;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  writeRegister(0x6B, 0x00);  // Wake the MPU6050.
  delay(100);

  writeRegister(0x1C, 0x10);  // Accelerometer: +/-8g.
  writeRegister(0x1B, 0x08);  // Gyroscope: +/-500 dps.
  delay(50);

  calibrateGyro();

  Serial.println(F("MPU6050 fall detector ready."));
}

void loop() {
  unsigned long now = millis();

  if ((long)(now - pauseUntil) < 0) {
    return;
  }

  if (now - lastSample < SAMPLE_TIME) {
    return;
  }

  lastSample = now;

  Reading r;

  if (!readSensor(r)) {
    return;
  }

  checkFall(r, now);

  if (now - lastPrint >= PRINT_TIME) {
    lastPrint = now;
    printReading(r);
  }
}