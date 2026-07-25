#include <Arduino.h>
#include <Wire.h>

static const uint8_t MPU6050_ADDRESS = 0x68;
static const uint8_t PWR_MGMT_1 = 0x6B;
static const uint8_t GYRO_CONFIG = 0x1B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t ACCEL_XOUT_H = 0x3B;

static const float ACCEL_LSB_PER_G = 4096.0f;  // +/-8g
static const float GYRO_LSB_PER_DPS = 65.5f;   // +/-500 dps

static const unsigned long SAMPLE_INTERVAL_MS = 20;
static const unsigned long PRINT_INTERVAL_MS = 1000;
static const unsigned long COOLDOWN_MS = 10000;

enum FallState {
  NORMAL,
  POSSIBLE_FREE_FALL,
  IMPACT_DETECTED,
  POST_IMPACT_MONITORING
};

struct MotionSample {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float accelMagnitude;
  float gyroMagnitude;
};

FallState fallState = NORMAL;

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

unsigned long lastSampleMs = 0;
unsigned long lastPrintMs = 0;
unsigned long stateStartedMs = 0;
unsigned long stillSinceMs = 0;
unsigned long cooldownUntilMs = 0;

float impactPeakG = 0.0f;
float gyroPeakDps = 0.0f;
float orientationChangeDeg = 0.0f;

static float absf(float value) {
  return value < 0.0f ? -value : value;
}

static float squareRoot(float value) {
  if (value <= 0.0f) {
    return 0.0f;
  }

  float estimate = value > 1.0f ? value : 1.0f;

  for (uint8_t i = 0; i < 16; ++i) {
    estimate = 0.5f * (estimate + value / estimate);
  }

  return estimate;
}

static int16_t joinBytes(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)((uint16_t)highByte << 8 | lowByte);
}

static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readMotion(MotionSample &sample) {
  uint8_t data[14];

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14);

  if (received != 14) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < 14; ++i) {
    if (!Wire.available()) {
      return false;
    }
    data[i] = Wire.read();
  }

  int16_t rawAx = joinBytes(data[0], data[1]);
  int16_t rawAy = joinBytes(data[2], data[3]);
  int16_t rawAz = joinBytes(data[4], data[5]);

  int16_t rawGx = joinBytes(data[8], data[9]);
  int16_t rawGy = joinBytes(data[10], data[11]);
  int16_t rawGz = joinBytes(data[12], data[13]);

  sample.ax = (float)rawAx / ACCEL_LSB_PER_G;
  sample.ay = (float)rawAy / ACCEL_LSB_PER_G;
  sample.az = (float)rawAz / ACCEL_LSB_PER_G;

  sample.gx = ((float)rawGx / GYRO_LSB_PER_DPS) - gyroOffsetX;
  sample.gy = ((float)rawGy / GYRO_LSB_PER_DPS) - gyroOffsetY;
  sample.gz = ((float)rawGz / GYRO_LSB_PER_DPS) - gyroOffsetZ;

  sample.accelMagnitude = squareRoot(
      sample.ax * sample.ax +
      sample.ay * sample.ay +
      sample.az * sample.az);

  sample.gyroMagnitude = squareRoot(
      sample.gx * sample.gx +
      sample.gy * sample.gy +
      sample.gz * sample.gz);

  return true;
}

static void calibrateGyroscope() {
  const uint8_t sampleCount = 120;
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  uint8_t goodSamples = 0;

  Serial.println(F("Keep MPU6050 still: calibrating gyro..."));

  for (uint8_t i = 0; i < sampleCount; ++i) {
    MotionSample sample;

    if (readMotion(sample)) {
      sumX += sample.gx;
      sumY += sample.gy;
      sumZ += sample.gz;
      ++goodSamples;
    }

    delay(SAMPLE_INTERVAL_MS);
  }

  if (goodSamples > 0) {
    gyroOffsetX = sumX / goodSamples;
    gyroOffsetY = sumY / goodSamples;
    gyroOffsetZ = sumZ / goodSamples;
  }

  Serial.println(F("Gyro calibration complete."));
}

static void resetEventValues() {
  impactPeakG = 0.0f;
  gyroPeakDps = 0.0f;
  orientationChangeDeg = 0.0f;
  stillSinceMs = 0;
}

static void printLiveReadings(const MotionSample &sample) {
  Serial.print(F("ACCEL: X="));
  Serial.print(sample.ax, 2);
  Serial.print(F("g Y="));
  Serial.print(sample.ay, 2);
  Serial.print(F("g Z="));
  Serial.print(sample.az, 2);

  Serial.print(F("g | GYRO: X="));
  Serial.print(sample.gx, 1);
  Serial.print(F(" Y="));
  Serial.print(sample.gy, 1);
  Serial.print(F(" Z="));
  Serial.println(sample.gz, 1);
}

static void reportFall() {
  int confidence = 55;

  if (impactPeakG >= 3.0f) {
    confidence += 15;
  }
  if (gyroPeakDps >= 150.0f) {
    confidence += 15;
  }
  if (orientationChangeDeg >= 70.0f) {
    confidence += 15;
  }
  if (confidence > 100) {
    confidence = 100;
  }

  Serial.println(F("FALL_DETECTED"));
  Serial.print(F("Impact="));
  Serial.print(impactPeakG, 2);
  Serial.print(F("g | GyroPeak="));
  Serial.print(gyroPeakDps, 1);
  Serial.print(F(" dps | OrientationChange="));
  Serial.print(orientationChangeDeg, 1);
  Serial.print(F(" deg | Confidence="));
  Serial.print(confidence);
  Serial.println(F("%"));
}

static void updateFallDetection(const MotionSample &sample, unsigned long now) {
  // Low thresholds for easy testing. Raise these later to reduce false alarms.
  const float freeFallG = 0.90f;
  const float impactG = 1.25f;
  const float orientationRequired = 8.0f;
  const float stillAccelTolerance = 0.45f;
  const float stillGyroDps = 70.0f;

  if (sample.gyroMagnitude > gyroPeakDps) {
    gyroPeakDps = sample.gyroMagnitude;
  }

  if (sample.accelMagnitude > impactPeakG) {
    impactPeakG = sample.accelMagnitude;
  }

  switch (fallState) {
    case NORMAL:
      if (sample.accelMagnitude < freeFallG) {
        resetEventValues();

        impactPeakG = sample.accelMagnitude;
        gyroPeakDps = sample.gyroMagnitude;
        stateStartedMs = now;

        fallState = POSSIBLE_FREE_FALL;
      }
      break;

    case POSSIBLE_FREE_FALL:
      // Count rotation throughout the falling movement, before impact.
      orientationChangeDeg += sample.gyroMagnitude *
                              (SAMPLE_INTERVAL_MS / 1000.0f);

      if (sample.accelMagnitude >= impactG) {
        impactPeakG = sample.accelMagnitude;
        stateStartedMs = now;

        fallState = IMPACT_DETECTED;
      } else if (now - stateStartedMs > 2200) {
        fallState = NORMAL;
      }
      break;

    case IMPACT_DETECTED:
      orientationChangeDeg += sample.gyroMagnitude *
                              (SAMPLE_INTERVAL_MS / 1000.0f);

      if (now - stateStartedMs > 80) {
        stateStartedMs = now;
        stillSinceMs = 0;

        fallState = POST_IMPACT_MONITORING;
      }
      break;

    case POST_IMPACT_MONITORING: {
      orientationChangeDeg += sample.gyroMagnitude *
                              (SAMPLE_INTERVAL_MS / 1000.0f);

      bool isStill =
          absf(sample.accelMagnitude - 1.0f) <= stillAccelTolerance &&
          sample.gyroMagnitude <= stillGyroDps;

      if (isStill) {
        if (stillSinceMs == 0) {
          stillSinceMs = now;
        }

        if (now - stillSinceMs >= 400 &&
            orientationChangeDeg >= orientationRequired) {
          reportFall();

          // Stop all MPU6050 reads and serial live output for 10 seconds.
          cooldownUntilMs = now + COOLDOWN_MS;
          fallState = NORMAL;
        }
      } else {
        stillSinceMs = 0;
      }

      if (now - stateStartedMs > 4500) {
        fallState = NORMAL;
      }

      break;
    }
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  // Wake the MPU6050.
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Set accelerometer to +/-8g.
  writeRegister(ACCEL_CONFIG, 0x10);

  // Set gyroscope to +/-500 dps.
  writeRegister(GYRO_CONFIG, 0x08);
  delay(50);

  calibrateGyroscope();

  Serial.println(F("MPU6050 fall detector ready."));
}

void loop() {
  unsigned long now = millis();

  // After FALL_DETECTED, do not read the MPU6050 for 10 seconds.
  if ((long)(now - cooldownUntilMs) < 0) {
    return;
  }

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }

  lastSampleMs = now;

  MotionSample sample;

  if (!readMotion(sample)) {
    return;
  }

  updateFallDetection(sample, now);

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printLiveReadings(sample);
  }
}