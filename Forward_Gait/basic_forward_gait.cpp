// Basic Gait for learning purpose, below code has technical errors (currently working it)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// setting servo pulse limit
#define SERVOMIN 120
#define SERVOMAX 520

// robot leg lengths
const float L1 = 18.0;   // thigh (cm)
const float L2 = 22.0;   // shank (cm)

// gait parameters
const float BODY_HEIGHT = 32.0;
const float STEP_LENGTH = 16.0;  
const float STEP_HEIGHT = 10.0;   // if you want to move robot on stairs then you have to work on this line or look at gait parameters (if you want your robot to move legs according)
const float SPEED = 0.8;    // Hz

// gait time
float gait_time = 0.0;
unsigned long lastMillis = 0;

// helper function to convert angle in servo pulse
int angleToPulse(float angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  return map(angleDeg, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(9600);  // baud rate set according to your board
  Wire.begin();   // for i2c communication (since pca9685 in which all servos connected works on i2c communication so)
  pwm.begin();   
  pwm.setPWMFreq(50);  // setting pwm frequency

  // This is standing position or neutral position
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
  pwm.setPWM(10,  0, angleToPulse(180-145));
  pwm.setPWM(11,  0, angleToPulse(180-120));

  lastMillis = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastMillis) / 1000.0;
  lastMillis = now;

  if (1) {
    gait_time += dt;
  }

  float basePhase = 2 * PI * SPEED * gait_time;

  // selecting which leg have what phase according to gait like walking, trotting, running
  for (int leg = 0; leg < 4; leg++) {

    bool diagonalA = (leg == 0 || leg == 3); // FL & RR
    float phi = diagonalA ? basePhase : basePhase + PI;

    float phase = fmod(phi, 2 * PI);
    float phaseNorm = phase / (2 * PI); // 0 → 1

    float x, z;

    // stance and swing, (for how much time leg on ground and in air)
    if (phaseNorm < 0.5) {
      // for how much time leg keep toching the ground while legs moving bachward to push body forward
      float t = phaseNorm / 0.5;

      x = (1 - t) * ( -STEP_LENGTH / 2)
        + t * (STEP_LENGTH / 2);    // x = toe moving forward or backward

      z = BODY_HEIGHT;   // z = toe moving up and down
    } 
    else {
      // for how much time leg will be in air while legs moving forward
      float t = (phaseNorm - 0.5) / 0.5;

      x = (1 - t) * (STEP_LENGTH / 2)  
        + t * ( -STEP_LENGTH / 2);

      z = BODY_HEIGHT - STEP_HEIGHT * sin(t * PI);  
    }

    // Inverse Kinematics
    float D = (x*x + z*z - L1*L1 - L2*L2) / (2 * L1 * L2);
    D = constrain(D, -1.0, 1.0);

    float knee = acos(D);
    float hip = atan2(z, x) -
                atan2(L2 * sin(knee),
                      L1 + L2 * cos(knee));

    float hipDeg  = hip  * 180.0 / PI;
    float kneeDeg = knee * 180.0 / PI;

    // channel mapping (each leg have 3 servo so multiple of 3)
    int hipChannel  = leg * 3;
    int hipChannel2  = leg * 3 + 1;
    int kneeChannel = leg * 3 + 2;

    if (leg==0 || leg==3){
      if(leg==0){
        pwm.setPWM(hipChannel,  0, angleToPulse(30));\
        pwm.setPWM(hipChannel2,  0, angleToPulse(145-hipDeg));
        pwm.setPWM(kneeChannel, 0, angleToPulse(60-kneeDeg));
        Serial.print("Leg 0 = ");
        Serial.print(hipDeg);
        Serial.print(" , ");
        Serial.println(kneeDeg);
      }
      else{
        pwm.setPWM(hipChannel,  0, angleToPulse(30));
        pwm.setPWM(hipChannel2,  0, angleToPulse(180-145+hipDeg));
        pwm.setPWM(kneeChannel, 0, angleToPulse(180-120+kneeDeg));
        Serial.print("Leg 3 = ");
        Serial.print(hipDeg);
        Serial.print(" , ");
        Serial.println(kneeDeg);
      }
    }
    else{
      if(leg==1){
        pwm.setPWM(hipChannel,  0, angleToPulse(180-30));
        pwm.setPWM(hipChannel2,  0, angleToPulse(180-140+hipDeg));
        pwm.setPWM(kneeChannel, 0, angleToPulse(180-125-kneeDeg));
        Serial.print("Leg 1 = ");
        Serial.print(hipDeg);
        Serial.print(" , ");
        Serial.println(kneeDeg);
      }
      else{
        pwm.setPWM(hipChannel,  0, angleToPulse(180-30));
        pwm.setPWM(hipChannel2,  0, angleToPulse(145-hipDeg));
        pwm.setPWM(kneeChannel, 0, angleToPulse(60-kneeDeg));
        Serial.print("Leg 2 = ");
        Serial.print(hipDeg);
        Serial.print(" , ");
        Serial.println(kneeDeg);
      }
    }
    
  }

  delay(10); // ~100 Hz
}
