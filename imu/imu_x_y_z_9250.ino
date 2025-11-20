#include "Wire.h"
#include <Servo.h>
#include <MPU9250_WE.h> // Include the new MPU-9250 library

// --- MPU-9250 Setup ---
#define MPU9250_ADDR 0x68 
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR); 

// --- Servo Setup (Unchanged) ---
Servo myservo1; 
Servo myservo2; 
Servo myservo3; 
const int servoPin1 = 8;
const int servoPin2 = 9;
const int servoPin3 = 10;

// --- Kinematics & Filter ---
float q[4] = {1.0, 0.0, 0.0, 0.0}; 
float segKappa = 0.0, segPhi = 0.0;
float segmentLength = 0.14; 
float Kp = 30.0, Ki = 0.0;

// <-- MODIFIED: Re-added gscale from your original code.
// This is the correct way to convert raw gyro data to rad/s for the Mahony filter
// when the range is +/- 250 dps.
#define gscale ((250.0 / 32768.0) * (PI / 180.0))

// --- Timers & State Machine (Unchanged) ---
unsigned long lastServoMoveTime = 0;
unsigned long lastPrintTime = 0;
const long servoMoveInterval = 400; 
const long printInterval = 250;     
int servoAngle = 0; 
enum ServoState {
  STATE_SERVO_1,
  STATE_SERVO_2,
  STATE_SERVO_3,
  STATE_COMBINATION,
  STATE_RESET
};
ServoState servoSequenceState = STATE_SERVO_1;


void setup() {
  Wire.begin();
  Serial.begin(115200); 
  Serial.println("Starting setup...");

  // --- Initialize MPU-9250 ---
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
    while (1) {} 
  } else {
    Serial.println("MPU9250 is connected");
  }

  // --- Apply Calibration & Settings from your "Code 2" ---
  // !! IMPORTANT !!
  // Replace these placeholder values with your sensor's actual offsets
  // found by running the calibration sketch (Code 2).
  myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);

  // Set ranges (250dps matches your 'gscale' calculation)
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  // Set Digital Low Pass Filters (DLPF)
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6); 
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6); 
  Serial.println("MPU-9250 configured.");

  // --- Servo Setup (Unchanged) ---
  myservo1.attach(servoPin1); 
  myservo2.attach(servoPin2); 
  myservo3.attach(servoPin3); 
  myservo1.write(0); 
  myservo2.write(0); 
  myservo3.write(0);   

  delay(1000); 
  Serial.println("Setup complete!");
}

void loop() {
  // === Part 1: High-Frequency IMU Reading and Filtering ===
  
  // --- Deltat Calculation (Unchanged) ---
  static unsigned long now = 0, last = 0;
  now = micros();
  float deltat = (now - last) * 1.0e-6;
  last = now;

  // --- MODIFIED: MPU9250 Reading (FIXED) ---
  // These functions get the raw sensor values *after* the library
  // has applied the offsets you set in setup().
  xyzFloat acc = myMPU9250.getCorrectedAccRawValues(); // <-- FIXED
  xyzFloat gyr = myMPU9250.getCorrectedGyrRawValues(); // <-- FIXED

  float Axyz[3], Gxyz[3];

  // Accelerometer data:
  // The Mahony filter normalizes this vector anyway.
  // Using the corrected raw values is exactly what your original MPU-6050 code did
  // (since its scaling factor was 1.0).
  Axyz[0] = acc.x;
  Axyz[1] = acc.y;
  Axyz[2] = acc.z;
  
  // Gyroscope data:
  // We must convert the corrected raw values to RADIANS/second.
  // We use the 'gscale' factor, which is identical to your MPU-6050 code.
  Gxyz[0] = gyr.x * gscale;
  Gxyz[1] = gyr.y * gscale;
  Gxyz[2] = gyr.z * gscale;

  // --- Run Filter (Unchanged) ---
  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  // --- Kinematic Calculations (Unchanged) ---
  float rawKappa = acos(1 - 2 * (pow(q[2], 2) + pow(q[1], 2))) / segmentLength;
  segKappa = constrain(rawKappa, 0.0001, 10);
  segPhi = wrapToPI(-atan2((q[1] * q[0]) + (q[2] * q[3]), (q[2] * q[0]) - (q[1] * q[3])));

  // --- Calculate (x, y, z) (Unchanged) ---
  float x, y, z;
  if (segKappa < 0.0001) {
    x = 0.0; y = 0.0; z = segmentLength;
  } else {
    float theta = segKappa * segmentLength; 
    float r = 1.0 / segKappa;             
    x = r * (1.0 - cos(theta)) * cos(segPhi);
    y = r * (1.0 - cos(theta)) * sin(segPhi);
    z = r * sin(theta);
  }


  // === Part 2: Timed Actions (No Delays) ===
  // (This entire section is unchanged)
  
  unsigned long currentTime = millis();

  // --- Timed Servo Control (State Machine) ---
  if (currentTime - lastServoMoveTime >= servoMoveInterval) {
    lastServoMoveTime = currentTime; 

    // --- STATE ACTION ---
    if (servoSequenceState == STATE_RESET) {
       // Do nothing
    } else {
      switch (servoSequenceState) {
        case STATE_SERVO_1:
          myservo1.write(servoAngle);
          break;
        case STATE_SERVO_2:
          myservo2.write(servoAngle);
          break;
        case STATE_SERVO_3:
          myservo3.write(servoAngle);
          break;
        case STATE_COMBINATION:
          myservo1.write(servoAngle);
          myservo2.write(servoAngle);
          myservo3.write(servoAngle);
          break;
      }
      servoAngle += 5;
    }

    // --- STATE TRANSITION ---
    if (servoSequenceState == STATE_RESET) {
      servoSequenceState = STATE_SERVO_1;
      
    } else if (servoAngle > 180) {
      servoAngle = 0; 
      switch (servoSequenceState) {
        case STATE_SERVO_1:
          myservo1.write(0); 
          servoSequenceState = STATE_SERVO_2;
          break;
        case STATE_SERVO_2:
          myservo2.write(0); 
          servoSequenceState = STATE_SERVO_3;
          break;
        case STATE_SERVO_3:
          myservo3.write(0); 
          servoSequenceState = STATE_COMBINATION;
          break;
        case STATE_COMBINATION:
          myservo1.write(0);
          myservo2.write(0);
          myservo3.write(0);
          servoSequenceState = STATE_RESET; 
          break;
      }
    }
  }

  // --- Timed Serial Printing ---
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime; 

    Serial.print("Servo State: "); Serial.println(servoSequenceState);
    Serial.print("Servo Angles (1,2,3): "); 
    Serial.print(myservo1.read());       
    Serial.print(", ");                     
    Serial.print(myservo2.read());       
    Serial.print(", ");                     
    Serial.println(myservo3.read());     
    
    Serial.print("Kappa: "); Serial.println(segKappa, 6);
    Serial.print("Phi: "); Serial.println(segPhi, 6);
    Serial.print("X: "); Serial.println(x, 6);
    Serial.print("Y: "); Serial.println(y, 6);
    Serial.print("Z: "); Serial.println(z, 6);
    Serial.println("-------------");
  }
}

/**
 * @brief Mahony AHRS sensor fusion algorithm
 * (This function is unchanged)
 */
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm, vx, vy, vz, ex, ey, ez;
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;

  float tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0) {
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;
      gy += iy;
      gz += iz;
    }

    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  deltat *= 0.5;
  gx *= deltat;
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) q[i] *= recipNorm;
}

/**
 * @brief Wraps an angle in radians to the range [-PI, PI]
 * (This function is unchanged)
 */
double wrapToPI(double angle) {
  return fmod(angle + PI, 2 * PI) - PI;
}