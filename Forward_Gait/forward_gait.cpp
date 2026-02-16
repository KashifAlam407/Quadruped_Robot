// Currently i have implemented only forward gait and moving quadruped with manually/remote like forward, bachward, left and right

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------- Servo limits ----------------
#define SERVOMIN 120
#define SERVOMAX 520

// ---------------- Geometry (cm) ----------------
const float L1 = 18.0;   // thigh
const float L2 = 23.0;   // shank

// ---------------- Gait parameters ----------------
const float BODY_HEIGHT = 33.0;
const float STEP_LENGTH = 14.0;
const float STEP_HEIGHT = 6.0;
const float SPEED = 2;   // Hz

// ---------------- Time ----------------
float gaitTime = 0;
unsigned long lastMillis = 0;

// ---------------- IK reference ----------------
float hip0 = 0, knee0 = 0;
bool calibrated = false;

// ---------------- Helpers ----------------
int angleToPulse(float ang) {
  ang = constrain(ang, 0, 180);
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}

// Neutral pose or Standing Pose
void standPose() {
  // Front Left
  pwm.setPWM(0,  0, angleToPulse(30));
  pwm.setPWM(1,  0, angleToPulse(145));
  pwm.setPWM(2,  0, angleToPulse(60));

  // Front Right
  pwm.setPWM(3,  0, angleToPulse(180-30));
  pwm.setPWM(4,  0, angleToPulse(180-140));
  pwm.setPWM(5,  0, angleToPulse(180-125));

  // Rear Left
  pwm.setPWM(6,  0, angleToPulse(180-30));
  pwm.setPWM(7,  0, angleToPulse(145));
  pwm.setPWM(8,  0, angleToPulse(60));

  // Rear Right
  pwm.setPWM(9,  0, angleToPulse(30));
  pwm.setPWM(10, 0, angleToPulse(180-145));
  pwm.setPWM(11, 0, angleToPulse(180-120));
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  standPose();
  delay(2000);

  lastMillis = millis();
}

void loop() {

  unsigned long now = millis();
  float dt = (now - lastMillis) / 1000.0;
  lastMillis = now;
  gaitTime += dt;

  float basePhase = 2 * PI * SPEED * gaitTime;

  // setting phase diff between legs according to gait like trot, walking, running
  for (int leg = 0; leg < 4; leg++) {

    // Trot: diagonal pairs
    bool diagA = (leg == 0 || leg == 3); // FL & RR
    float phase = diagA ? basePhase : basePhase + PI;
    phase = fmod(phase, 2 * PI);
    float p = phase / (2 * PI);

    float dx, dz;

    // stance and swing (we can change the ratio or swing and stance, to look like real walking, trotting)
    if (p < 0.5) {
      // stance: foot moves backward (how much time it will be touching ground)
      float t = p / 0.5;
      dx = (1 - t) * ( STEP_LENGTH / 2)
         + t       * (-STEP_LENGTH / 2);
      dz = 0;
    } else {
      // swing: foot moves forward + lift (how much time it will be in air)
      float t = (p - 0.5) / 0.5;
      dx = (1 - t) * (-STEP_LENGTH / 2)
         + t       * ( STEP_LENGTH / 2);
      dz = -STEP_HEIGHT * sin(t * PI);
    }

    // 
    const float FOOT_X_OFFSET = 6.0;

    // absolute foot position
    float z =  FOOT_X_OFFSET + dx;

    // float x = dx;
    float x = BODY_HEIGHT + dz;

    // inverse kinematics
    float D = (x*x + z*z - L1*L1 - L2*L2) / (2 * L1 * L2);
    D = constrain(D, -1.0, 1.0);

    float knee = acos(D);
    float hip  = atan2(z, x) -
                 atan2(L2 * sin(knee),
                       L1 + L2 * cos(knee));

    float hipDeg  = hip  * 180.0 / PI;
    float kneeDeg = knee * 180.0 / PI;

    // capture stand Inverse Kinematics once
    if (!calibrated) {
      hip0 = hipDeg;
      knee0 = kneeDeg;
      calibrated = true;
    }

    float hipRel  = hipDeg  - hip0;
    float kneeRel = kneeDeg - knee0;

    // apply angles to servos
    if (leg == 0) { // Front Left
      pwm.setPWM(1, 0, angleToPulse(145 - hipRel-15));
      pwm.setPWM(2, 0, angleToPulse(60  - kneeRel));
    }
    else if (leg == 1) { // Front Right
      pwm.setPWM(4, 0, angleToPulse(180-140 + hipRel+15));
      pwm.setPWM(5, 0, angleToPulse(180-125 + kneeRel));
    }
    else if (leg == 2) { // Rear Left
      pwm.setPWM(7, 0, angleToPulse(145 - hipRel-15));
      pwm.setPWM(8, 0, angleToPulse(60  - kneeRel));
    }
    else { // Rear Right
      pwm.setPWM(10, 0, angleToPulse(180-145 + hipRel+15));
      pwm.setPWM(11, 0, angleToPulse(180-120 + kneeRel));
    }
  }

  delay(10); // ~100 Hz
}
