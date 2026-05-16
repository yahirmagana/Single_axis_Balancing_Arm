#include <Wire.h>
#include <Servo.h>

// --- Configuration ---
#define ESC_PIN 9
#define IMU_ADDRESS 0x69 

// --- Calibration ---
float roll_offset = 0.0; 

// --- PID Constants ---
float Kp = 0.039; 
float Ki = 0.04;
float Kd = 0.16;

// --- System Variables ---
float setpoint = 0.0; 
float roll_angle = 0.0;
float base_throttle = 1247; 

// --- PID Variables ---
float error, last_error, integral, derivative, d_filtered, output;
unsigned long current_time, previous_time;
float dt;

// --- IMU Variables ---
float accel_x, accel_y, accel_z;
float gyro_x; 
float accel_roll;

Servo esc;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // ---------------------------------------------------------
  // 1. ESC ARMING SEQUENCE
  // ---------------------------------------------------------
  Serial.println("Arming ESC");
  esc.attach(ESC_PIN, 1000, 2000); 
  esc.writeMicroseconds(1000); 

  delay(5000); // 5 solid seconds of zero-throttle to arm it                 
  Serial.println("ESC Armed successfully.");

  // ---------------------------------------------------------
  // 2. IMU INITIALIZATION
  // ---------------------------------------------------------
  Serial.println("Initializing IMU");
  initIMU();
  Serial.println("IMU Ready. Starting PID control loop.");
  
  previous_time = millis();
}

void loop() {
  current_time = millis();
  dt = (current_time - previous_time) / 1000.0;
  
  if (dt <= 0.0) return; 
  previous_time = current_time;

  readIMU();

  // ---------------------------------------------------------
  // 3. CALCULATE ROLL ANGLE
  // ---------------------------------------------------------
  accel_roll = (atan2(accel_y, accel_z) * 180.0 / PI) - roll_offset;
  roll_angle = 0.98 * (roll_angle + gyro_x * dt) + 0.02 * accel_roll;
  
  // If the arm flips too far, kill the motor
  if (roll_angle > 50) {
    esc.writeMicroseconds(1000);
    Serial.print("/*");
    Serial.println("Angle limit exceeded");
    Serial.println("*/");
    while(1); 
  }
  
  // ---------------------------------------------------------
  // 4. PID CALCULATIONS & OUTPUT LIMITING
  // ---------------------------------------------------------
  error = setpoint - roll_angle;
  
  if (abs(error) < 5.0) {
      integral += error * dt;
  } else {
    integral = 0; // Reset it if we are far away so it doesn't "wind up"
  }
  integral = constrain(integral, -50, 50); // Tighter constraint
  
  // This heavily smooths out the motor vibrations.
  float raw_derivative = (error - last_error) / dt;
  d_filtered = 0.9 * d_filtered + 0.1 * raw_derivative; 
  last_error = error;

  output = (Kp * error) + (Ki * integral) + (Kd * d_filtered);
  output = constrain(output, -120, 120);
  // ---------------------------------------------------------
  // 5. MOTOR OUTPUT
  // ---------------------------------------------------------
  float motor_pwm = base_throttle + output;
  motor_pwm = constrain(motor_pwm, 1000, 2000); // Final hardware safety guard
  
  esc.writeMicroseconds(motor_pwm);

  // TELEMETRY FOR SERIAL STUDIO
  // Format: /*Setpoint,Roll,Output*/
  Serial.print("/*");           // Start of frame
  Serial.print(setpoint);      Serial.print(",");
  Serial.print(roll_angle);    Serial.print(",");
  Serial.print(output);        Serial.print(",");
  Serial.print(motor_pwm);
  Serial.println("*/");          // End of frame
}
// --- Helper Functions ---

void initIMU() {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B); 
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readIMU() {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDRESS, 14, true); 

  if (Wire.available() == 14) {
    int16_t raw_accel_x = Wire.read() << 8 | Wire.read();
    int16_t raw_accel_y = Wire.read() << 8 | Wire.read();
    int16_t raw_accel_z = Wire.read() << 8 | Wire.read();
    
    //Wire.read(); Wire.read(); // Skip temperature
    
    int16_t raw_gyro_x = Wire.read() << 8 | Wire.read();
    int16_t raw_gyro_y = Wire.read() << 8 | Wire.read();
    int16_t raw_gyro_z = Wire.read() << 8 | Wire.read();

    accel_x = raw_accel_x / 16384.0;
    accel_y = raw_accel_y / 16384.0;
    accel_z = raw_accel_z / 16384.0;
    
    gyro_x = raw_gyro_x / 131.0; 
  }
}
