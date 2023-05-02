// Arduino libraries
#include <Wire.h> // I2C Library
#include <IBusBM.h> // iBUS Library
IBusBM IBus;

// iBUS
# define PITCH    0 // x-axis
# define ROLL     1 // y-axis
# define YAW      2 // z-axis
# define THROTTLE 3 // Throttle
# define ARM      4 // Arm switch
bool armed = false; // Arm switch position
int16_t input[5]; // iBUS channels

// Motors
# define M1 0 // Motor 1
# define M2 1 // Motor 2
# define M3 2 // Motor 3
# define M4 3 // Motor 4
const uint8_t motor[] = {3, 5, 6, 9}; // Motor pins
uint8_t output[4]; // Motor outputs
double rate[] = {200.0, 200.0, 400.0}; // Maximum rate in each axis

// MPU-6050 register addresses
# define MPU_6050 0x68 // MPU-6050 address
# define PWR_MGMT_1 0x6B // Power management register
# define SIGNAL_PATH_RESET 0x68 // Signal path reset register
# define CONFIG 0x1A // Configuration register
# define GYRO_CONFIG 0x1B // Gyroscope configuration register
# define TEMP_OUT 0x41 // Temperature measurement register
# define GYRO_OUT 0x43 // Gyroscope measurement register

// MPU-6050 variables
double gyro_Raw[3]; // Array to store MPU-6050 measurement in each axis
double gyro_Sensitivity = 32.8; // MPU-6050 sensitivity
double gyro_Temp; // MPU-6050 temperature
uint16_t gyro_Frequency = 1000; // MPU-6050 sampling frequency
double gyro_Elapsed_Time; // MPU-6050 sampling interval
unsigned long gyro_Prev_Time = 0; // MPU-6050 sampling time

// PD variables
uint16_t pd_Frequency = 1000; // PD loop frequency
double pd_Elapsed_Time; // PD loop interval
unsigned long pd_Prev_Time = 0; // PD loop time
double Kp[] = {0.75, 0.75, 0.84}; // Proportional gains
double Kd[] = {0.05, 0.05, 0.0}; // Derivative gains
double error[3]; // Errors
double prev_Error[3]; // Previous errors
double p[3]; // Proportional terms
double d[3]; // Derivative terms
double pd[3]; // PD outputs

// Setup routine
void setup() {
  Wire.begin(); // Initialise I2C bus
  Wire.setClock(400000); // Enable I2C 400kHz fast mode

  // Reset MPU-6050
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0b10000000);
  Wire.endTransmission();

  // Reset MPU-6050 signal paths
  Wire.beginTransmission(MPU_6050);
  Wire.write(SIGNAL_PATH_RESET);
  Wire.write(0b00000111);
  Wire.endTransmission();

  // Disable MPU-6050 low power sleep mode
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0b00000000);
  Wire.endTransmission();

  // Set MPU-6050 DLPF bandwidth to 42Hz
  Wire.beginTransmission(MPU_6050);
  Wire.write(CONFIG); // CONFIG register
  Wire.write(0b00000011);
  Wire.endTransmission();

  // Set MPU-6050 full-scale range to ±1000 deg/s
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_CONFIG);
  Wire.write(0b00010000);
  Wire.endTransmission();

  IBus.begin(Serial); // Initialise iBUS on UART RX pin

  // Configure each motor pin as an output
  for (uint8_t n = 0; n <= 3; n++) {
    pinMode(motor[n], OUTPUT);
  }
}


// Loop routine
void loop() {
  receive(); // Receive commands from radio transmitter
  gyro_Read(); // Measure angular velocity in each axis
  PD(); // Compute motor outputs using PD controller

  // Send PWM signal to each motor if armed, otherwise stop motors
  if (input[ARM] > 1000) {
    for (uint8_t n = 0; n <= 3; n++) {
      analogWrite(motor[n], output[n]);
    }
  } else {
    for (uint8_t n = 0; n <= 3; n++) {
      analogWrite(motor[n], 0);
    }
  }
}

// Receive function
void receive() {
  
  // Map 1000-2000 command from each channel to maximum angular velocity of ±800 deg/s
  input[ROLL] = map(IBus.readChannel(0), 1000, 2000, -rate[ROLL],rate[ROLL]);
  input[PITCH] = map(IBus.readChannel(1), 1000, 2000, rate[PITCH], -rate[PITCH]);
  input[YAW] = map(IBus.readChannel(3), 1000, 2000, rate[YAW], -rate[YAW]);

  // Map 1000-2000 command from throttle channel to 80% of PWM range
  input[THROTTLE] = map(IBus.readChannel(2), 1000, 2000, 0, 204);

  // Position of arm switch
  input[ARM] = IBus.readChannel(4);
}

// Gyroscope function
void gyro_Read() {
  
  gyro_Elapsed_Time = (micros() - gyro_Prev_Time) / 1000000.0; // Calculate elapsed time
  if (gyro_Elapsed_Time >= (1.0 / gyro_Frequency)) { // Obtain another measurement if the sampling period has elapsed

    // Measure MPU-6050 temperature
    Wire.beginTransmission(MPU_6050); // MPU-6050 address
    Wire.write(TEMP_OUT); // Temperature register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_6050, 2, true); // Request data
    gyro_Temp = Wire.read() << 8 | Wire.read(); // Obtain raw measurment
    gyro_Temp = (gyro_Temp / 340.0) + 36.53; // Equation to calculate actual temperature from raw measurement

    // Measure angular velocity in each axis
    Wire.beginTransmission(MPU_6050); // MPU-6050 address        
    Wire.write(GYRO_OUT); // Gyroscope register                    
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_6050, 6, true); // Request data
    for (uint8_t n = 0; n <= 2; n++) { // For each axis
      gyro_Raw[n] = Wire.read() << 8 | Wire.read(); // Obtain raw gyroscope measurement
      gyro_Raw[n] = (gyro_Raw[n] / gyro_Sensitivity) - offset(n, gyro_Temp); // Equation to calculate accurate measurement from raw measurement
    }
    gyro_Prev_Time = micros(); // Store current time
  }
}

// Temperature bias function
double offset(uint8_t axis, double temperature) {
  
  if (axis == PITCH) {
    return (-0.0007 * temperature) - 1.0825;
  } else if (axis == ROLL) {
    return (-0.0199 * temperature) + 2.6646;
  } else if (axis == YAW) {
    return (0.0217 * temperature) - 0.6035;
  }
}

// PD function
void PD() {
  
  pd_Elapsed_Time = (micros() - pd_Prev_Time) / 1000000.0; // Calculate elapsed time
  if (pd_Elapsed_Time >= (1.0 / pd_Frequency)) { // Calculate new output of PD controller if loop period has elapsed
    
    for (uint8_t n = 0; n <= 2; n++) { // For each axis
      error[n] = gyro_Raw[n] - input[n]; // Calculate error
      p[n] = Kp[n] * error[n]; // Calculate proportional term
      d[n] = Kd[n] * ((error[n] - prev_Error[n]) / pd_Elapsed_Time); // Calculate derivative term
      pd[n] = p[n] + d[n]; // Sum proportional and derivative terms
      pd[n] = constrain(pd[n], -51, 51); // Constrain output of PD controller to 20% of PWM range
      prev_Error[n] = error[n]; // Store current error
    }

    // Apply mixing matrix M to output of PD controller and constrain to PWM range
    output[M1] = constrain(input[THROTTLE] + pd[PITCH] + pd[ROLL] - pd[YAW], 0, 255);
    output[M2] = constrain(input[THROTTLE] - pd[PITCH] + pd[ROLL] + pd[YAW], 0, 255);
    output[M3] = constrain(input[THROTTLE] + pd[PITCH] - pd[ROLL] + pd[YAW], 0, 255);
    output[M4] = constrain(input[THROTTLE] - pd[PITCH] - pd[ROLL] - pd[YAW], 0, 255);
    
    pd_Prev_Time = micros(); // Store current time
  }
}
