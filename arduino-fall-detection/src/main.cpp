#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define DEBUG_MODE false

const unsigned long SAMPLE_INTERVAL_MS = 20;
const unsigned long CALIBRATION_TIME_MS = 2000;
const float FREE_FALL_THRESHOLD_G = 0.55f;
const float IMPACT_THRESHOLD_G = 2.40f;
const float ORIENTATION_CHANGE_THRESHOLD_DEG = 45.0f;
const float IMPACT_GYRO_THRESHOLD_DPS = 80.0f;
const float INACTIVITY_GYRO_THRESHOLD_DPS = 18.0f;
const float INACTIVITY_ACCEL_TOLERANCE_G = 0.14f;
const float QUIET_GYRO_THRESHOLD_DPS = 8.0f;
const unsigned long FREE_FALL_TIMEOUT_MS = 700;
const unsigned long ORIENTATION_TIMEOUT_MS = 1200;
const unsigned long POST_IMPACT_WINDOW_MS = 1800;
const unsigned long EVENT_COOLDOWN_MS = 3000;
const float MAGNITUDE_FILTER_ALPHA = 0.35f;
const float ANGLE_FILTER_ALPHA = 0.20f;
const float BASELINE_ADAPT_ALPHA = 0.01f;
const float GRAVITY_MPS2 = 9.80665f;

Adafruit_MPU6050 mpu;

enum DetectionState {
  NORMAL,
  POSSIBLE_FREE_FALL,
  IMPACT_DETECTED,
  POST_EVENT_MONITORING
};

struct Motion {
  float accelG;
  float gyroDps;
  float rollDeg;
  float pitchDeg;
};

DetectionState state = NORMAL;
Motion filtered = {1.0f, 0.0f, 0.0f, 0.0f};

float baselineRollDeg = 0.0f;
float baselinePitchDeg = 0.0f;
float impactPeakG = 0.0f;
float impactPeakGyroDps = 0.0f;

unsigned long lastSampleMs = 0;
unsigned long stateStartMs = 0;
unsigned long lastEventMs = 0;

unsigned int quietSamples = 0;
unsigned int postImpactSamples = 0;

bool primed = false;

float ema(float oldValue, float sample, float alpha) {
  return oldValue + alpha * (sample - oldValue);
}

float angleDifference(float a, float b) {
  float d = a - b;

  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;

  return d;
}

float orientationChange() {
  float roll = angleDifference(filtered.rollDeg, baselineRollDeg);
  float pitch = filtered.pitchDeg - baselinePitchDeg;

  return sqrt(roll * roll + pitch * pitch);
}

void readMotion() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelG = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  ) / GRAVITY_MPS2;

  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;

  float pitch = atan2(
    -a.acceleration.x,
    sqrt(
      a.acceleration.y * a.acceleration.y +
      a.acceleration.z * a.acceleration.z
    )
  ) * 180.0f / PI;

  float gyroDps = sqrt(
    g.gyro.x * g.gyro.x +
    g.gyro.y * g.gyro.y +
    g.gyro.z * g.gyro.z
  ) * 180.0f / PI;

  if (!primed) {
    filtered = {accelG, gyroDps, roll, pitch};
    primed = true;
    return;
  }

  filtered.accelG = ema(filtered.accelG, accelG, MAGNITUDE_FILTER_ALPHA);
  filtered.gyroDps = ema(filtered.gyroDps, gyroDps, MAGNITUDE_FILTER_ALPHA);
  filtered.rollDeg = ema(filtered.rollDeg, roll, ANGLE_FILTER_ALPHA);
  filtered.pitchDeg = ema(filtered.pitchDeg, pitch, ANGLE_FILTER_ALPHA);
}

void setState(DetectionState next, unsigned long now) {
  state = next;
  stateStartMs = now;

  if (next == POST_EVENT_MONITORING) {
    quietSamples = 0;
    postImpactSamples = 0;
  }
}

void sendEvent(unsigned long now, float orientationDeg) {
  float confidence =
    0.55f +
    min(0.18f, (impactPeakG - IMPACT_THRESHOLD_G) * 0.08f) +
    min(0.17f, (orientationDeg - ORIENTATION_CHANGE_THRESHOLD_DEG) * 0.003f) +
    min(0.10f, (float)quietSamples / postImpactSamples * 0.10f);

  Serial.print(F("FALL_EVENT,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(impactPeakG, 2);
  Serial.print(',');
  Serial.print(impactPeakGyroDps, 1);
  Serial.print(',');
  Serial.print(orientationDeg, 1);
  Serial.print(',');
  Serial.println(constrain(confidence, 0.0f, 0.99f), 2);
}

void updateDetector(unsigned long now) {
  float orientationDeg = orientationChange();

  if (state == NORMAL && filtered.gyroDps < QUIET_GYRO_THRESHOLD_DPS) {
    baselineRollDeg = ema(
      baselineRollDeg,
      filtered.rollDeg,
      BASELINE_ADAPT_ALPHA
    );

    baselinePitchDeg = ema(
      baselinePitchDeg,
      filtered.pitchDeg,
      BASELINE_ADAPT_ALPHA
    );
  }

  switch (state) {
    case NORMAL:
      if (
        now - lastEventMs >= EVENT_COOLDOWN_MS &&
        filtered.accelG <= FREE_FALL_THRESHOLD_G
      ) {
        setState(POSSIBLE_FREE_FALL, now);
      }
      break;

    case POSSIBLE_FREE_FALL:
      if (
        filtered.accelG >= IMPACT_THRESHOLD_G &&
        filtered.gyroDps >= IMPACT_GYRO_THRESHOLD_DPS
      ) {
        impactPeakG = filtered.accelG;
        impactPeakGyroDps = filtered.gyroDps;
        setState(IMPACT_DETECTED, now);
      } else if (now - stateStartMs > FREE_FALL_TIMEOUT_MS) {
        setState(NORMAL, now);
      }
      break;

    case IMPACT_DETECTED:
      impactPeakG = max(impactPeakG, filtered.accelG);
      impactPeakGyroDps = max(impactPeakGyroDps, filtered.gyroDps);

      if (orientationDeg >= ORIENTATION_CHANGE_THRESHOLD_DEG) {
        setState(POST_EVENT_MONITORING, now);
      } else if (now - stateStartMs > ORIENTATION_TIMEOUT_MS) {
        setState(NORMAL, now);
      }
      break;

    case POST_EVENT_MONITORING:
      ++postImpactSamples;

      if (
        filtered.gyroDps <= INACTIVITY_GYRO_THRESHOLD_DPS &&
        abs(filtered.accelG - 1.0f) <= INACTIVITY_ACCEL_TOLERANCE_G
      ) {
        ++quietSamples;
      }

      if (now - stateStartMs >= POST_IMPACT_WINDOW_MS) {
        if (
          (float)quietSamples / postImpactSamples >= 0.70f &&
          orientationDeg >= ORIENTATION_CHANGE_THRESHOLD_DEG
        ) {
          sendEvent(now, orientationDeg);
          lastEventMs = now;
        }

        setState(NORMAL, now);
      }
      break;
  }
}

void calibrate() {
  float rollSum = 0.0f;
  float pitchSum = 0.0f;
  unsigned int count = 0;
  unsigned long start = millis();

  while (millis() - start < CALIBRATION_TIME_MS) {
    readMotion();

    rollSum += filtered.rollDeg;
    pitchSum += filtered.pitchDeg;
    ++count;

    delay(SAMPLE_INTERVAL_MS);
  }

  baselineRollDeg = rollSum / count;
  baselinePitchDeg = pitchSum / count;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
#if DEBUG_MODE
    Serial.println(F("ERROR,MPU6050_NOT_FOUND"));
#endif
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrate();
  lastEventMs = millis() - EVENT_COOLDOWN_MS;
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;

  lastSampleMs = now;

  readMotion();
  updateDetector(now);
}