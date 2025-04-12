// Included Libraries
#include <PPM.h> // download from github https://github.com/VICLER/PPMDecoder/tree/master
#include <Adafruit_FXOS8700.h> // download from github https://github.com/adafruit/Adafruit_FXOS8700
#include <Adafruit_FXAS21002C.h> // download from github https://github.com/adafruit/Adafruit_FXAS21002C
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define CHANNELS 8   // max ppm channels
#define PPM_PIN 2    // receiver ppm pin
#define LEFT_PIN 13   
#define RIGHT_PIN 12
#define FORWARD_PIN 11
#define BACKWARD_PIN 10
#define SPIKE_PIN 9

// Assigning sensor addresses
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x1F);  // Default I2C address
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x21);  // Default I2C address

const float mass = 3; // Hex mass (Assumed constant)

// Variables for collecting the calibration data
float accelSum[3] = {0.0, 0.0, 0.0};
float gyroSum[3] = {0.0, 0.0, 0.0};

// Temporary variables for filtering
float accelPrev[3] = {0.0, 0.0, 0.0};
float accelLPF[3] = {0.0, 0.0, 0.0}; // For low-pass
float accelHPF[3] = {0.0, 0.0, 0.0}; // For high-pass
float gyroPrev[3] = {0.0, 0.0, 0.0};
float gyroLPF[3] = {0.0, 0.0, 0.0};
float gyroHPF[3] = {0.0, 0.0, 0.0};

// Pressure Sensor Configuration
const int sensorPin1 = A0; // Analog pin for the pressure sensor
const int sensorPin2 = A1; // Analog pin for second transducer
const float adcMax = 1023.0; // 10-bit ADC maximum value (Arduino)
const float vRef = 5.0;      // Arduino reference voltage (5V)

// Pressure sensor calibration
const float sensorMinVoltage = 0.5;  // Minimum sensor output voltage at 0 PSI
const float sensorMaxVoltage = 4.5;  // Maximum sensor output voltage at 300 PSI
const float pressureRange = 150.0;   // Pressure range (0-150 PSI)

float numReadings = 0;
bool Calibrated = false;

// Function to calculate pressure
float calculatePressure(float voltage) {
  return ((voltage - sensorMinVoltage) / (sensorMaxVoltage - sensorMinVoltage)) * pressureRange;
}

void resetIMU() {
  // Reset FXOS8700 (Accelerometer + Magnetometer)
  Wire.beginTransmission(0x1F); // FXOS8700 I2C address
  Wire.write(0x2B);              // CTRL_REG2 address for FXOS8700
  Wire.write(0x40);              // Set RST bit (bit 6)
  Wire.endTransmission();
  
  delay(50); // Wait for the reset to complete

  // Reset FXAS21002C (Gyroscope)
  Wire.beginTransmission(0x21); // FXAS21002C I2C address
  Wire.write(0x13);              // CTRL_REG1 address for FXAS21002C
  Wire.write(0x40);              // Set RST bit (bit 6)
  Wire.endTransmission();

  delay(50); // Wait for the reset to complete

  // Reinitialize the sensors with desired settings after reset
  accelmag.begin();
  gyro.begin();
}

void displaySensorDetails() {
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  sensor_t gyroSensor;
  gyro.getSensor(&gyroSensor);

  Serial.println("Sensor Details:");
  Serial.println("---------------");
  Serial.println("ACCELEROMETER");
  Serial.println(accel.name);
  Serial.print("Resolution: ");
  Serial.println(accel.resolution, 8);
  
  Serial.println("MAGNETOMETER");
  Serial.println(mag.name);
  
  Serial.println("GYROSCOPE");
  Serial.println(gyroSensor.name);
  Serial.println("---------------");

  delay(500);
}

// IMU data storage
float accelData[3] = {0, 0, 0};
float gyroData[3] = {0, 0, 0};
float momentum[3] = {0, 0, 0};
float accelFiltered[3] = {0.0, 0.0, 0.0};  // Filtered accelerometer data
float gyroFiltered[3] = {0.0, 0.0, 0.0};   // Filtered gyroscope data

// Calibration offsets
float accelOffset[3] = {0.0, 0.0, 0.0};  // Accelerometer offset
float gyroOffset[3] = {0.0, 0.0, 0.0};   // Gyroscope offset

// Integration variables (velocity calculation)
float velocity[3] = {0.0, 0.0, 0.0};  // Velocity in x, y, z
unsigned long lastUpdate = 0;
float deltat = 0.02;  // Assuming 50Hz update rate (20 ms)

// Low-pass filter variables
float alpha = 0.1;  // Low-pass filter constant

// Madgwick Filter variables
float beta = 0.1;  // Filter gain
float q[4] = {1.0, 0.0, 0.0, 0.0};  // Quaternion
float accelDataMadgwick[3], gyroDataMadgwick[3];

// Madgwick filter algorithm
void MadgwickFilter(float gx, float gy, float gz, float ax, float ay, float az) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float s1, s2, s3, s4;
  
  // Normalize accelerometer data
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Gyroscope data in radians/s
  gx = gx * DEG_TO_RAD;
  gy = gy * DEG_TO_RAD;
  gz = gz * DEG_TO_RAD;

  // Gradient descent algorithm (Madgwick's algorithm)
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4;
  float q3q3 = q3 * q3, q3q4 = q3 * q4;
  
  s1 = 2.0 * (q2q4 - q1q3 - ax);
  s2 = 2.0 * (q1q2 + q3q4 - ay);
  s3 = 2.0 * (q1q4 - q2q3 - az);
  
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3);
  s1 /= norm;
  s2 /= norm;
  s3 /= norm;
  
  s4 = -beta * (2.0 * (q2q4 - q1q3) + gx) - beta * (2.0 * (q1q2 + q3q4) + gy) - beta * (2.0 * (q1q4 - q2q3) + gz);

  q1 += s1 * deltat;
  q2 += s2 * deltat;
  q3 += s3 * deltat;
  q4 += s4 * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  q[0] = q1 / norm;
  q[1] = q2 / norm;
  q[2] = q3 / norm;
  q[3] = q4 / norm;
}

// High-pass filter variables
float prevAccelFiltered[3] = {0.0, 0.0, 0.0};  // Previous filtered accelerometer data
float beta_hp = 0.1;  // High-pass filter constant (adjust as needed)

void applyHighPassFilter() {
  for (int i = 0; i < 3; i++) {
    // High-pass filter formula: 
    // y[n] = beta * (y[n-1] + x[n] - x[n-1])
    float accelRaw = accelData[i];
    accelFiltered[i] = beta_hp * (accelFiltered[i] + accelRaw - prevAccelFiltered[i]);
    prevAccelFiltered[i] = accelRaw;  // Store the previous raw value for the next iteration
  }
}

unsigned long lastSensorReadTime = 0;
unsigned long lastSignalUpdateTime = 0;

// Chip Select Pin
const int chipSelect = 53;

// Function Prototypes
void calibrateSensors();
void processSensorData();

void setup() {
  ppm.begin(PPM_PIN, CHANNELS);
  pinMode(LEFT_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);
  pinMode(BACKWARD_PIN, OUTPUT);
  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(SPIKE_PIN, OUTPUT);
  Serial.begin(9600);
  SD.begin();

  // SD Card Checks

  if (SD.exists("Accel.csv")) {
    SD.remove("Accel.csv");
    Serial.println("Old Accel.csv deleted.");
  }

  if (SD.exists("Velocity.csv")) {
    SD.remove("Velocity.csv");
    Serial.println("Old Velocity.csv deleted");
  }

  if (SD.exists("Momentum.csv")) {
    SD.remove("Momentum.csv");
    Serial.println("Old Momentum.csv file deleted");
  }

  if (SD.exists("Pressure.csv")) {
    SD.remove("Pressure.csv");
    Serial.println("Old Pressure.csv file deleted");
  }

  // Creating New Data files
  File AccelData = SD.open("Accel.csv", FILE_WRITE);
  File VelocityData = SD.open("Velocity.csv", FILE_WRITE);
  File MomentumData = SD.open("Momentum.csv", FILE_WRITE);
  File PressureData = SD.open("Pressure.csv", FILE_WRITE);
  if(AccelData) {
    AccelData.println("Timestamp, Longitude (m/s2), Latitude (m/s2)"); // Headers
    AccelData.close();
  } else {
    Serial.println("Error creating Accel csv file");
  }
  if (VelocityData) {
    VelocityData.println("Timestamp, Longitude (m/s), Latitude(m/s)");
    VelocityData.close();
  } else {
    Serial.println("Error opening velocity csv file");
  }
  if (MomentumData) {
    MomentumData.println("Timestamp, Longitude (N-m), Latitude (N-m)");
    MomentumData.close();
  } else {
    Serial.println("Error opening momentum csv file");
  }
  if (PressureData) {
    PressureData.println("Timestamp, Canister1, Canister2");
    PressureData.close();
  } else {
    Serial.println("Error opening pressure csv file");
  }
  while (!Serial) { delay(10); }

  if (!accelmag.begin() || !gyro.begin()) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  resetIMU();

  accelmag.setOutputDataRate(ODR_50HZ);
  accelmag.setAccelRange(ACCEL_RANGE_4G);
  gyro.setODR(ODR_50HZ);
  gyro.setRange(GYRO_RANGE_250DPS);

  displaySensorDetails();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long timestamp = millis();

  int CH1 = ppm.get(1); // Read the first PPM channel
  int CH2 = ppm.get(2);
  int CH3 = ppm.get(3);
  int CH4 = ppm.get(4);
  int CH5 = ppm.get(5);
  int CH6 = ppm.get(6);
  int CH7 = ppm.get(7);
  int CH8 = ppm.get(8);

  // Pressure Readings
  int rawValue1 = analogRead(sensorPin1);
  int rawValue2 = analogRead(sensorPin2);
  float voltage1 = (rawValue1 / adcMax) * vRef;
  float voltage2 = (rawValue2 / adcMax) * vRef;
  float pressure1 = calculatePressure(voltage1);
  float pressure2 = calculatePressure(voltage2);

 // Write data to CSV
  File PressureData = SD.open("Pressure.csv", FILE_WRITE);
  if (PressureData) {
    PressureData.print(timestamp);
    PressureData.print(",");
    PressureData.print(pressure1, 2);
    PressureData.print(",");
    PressureData.println(pressure2,2);
    PressureData.close();
  } else {
    Serial.println("Error writing to pressure file.");
  }

  // Send control signals based on PPM channels
  if (currentTime - lastSignalUpdateTime >= 1000) { // Adjusted for signal update frequency
    lastSignalUpdateTime = currentTime;
    if (CH5 > 1750) { // 10 Hz 75% duty cycle
      if (CH6 > 1750) {
        if (CH1 < 1250 && CH2 < 1250) {
          digitalWrite(LEFT_PIN, HIGH);
          digitalWrite(BACKWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          delay(25);
        } else if (CH1 < 1250 && CH2 > 1750) {
          digitalWrite(LEFT_PIN, HIGH);
          digitalWrite(FORWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else if (CH1 > 1750 && CH2 < 1250) {
          digitalWrite(RIGHT_PIN, HIGH);
          digitalWrite(BACKWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else if (CH1 > 1750 && CH2 > 1750) {
          digitalWrite(RIGHT_PIN, HIGH);
          digitalWrite(FORWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else if (CH1 < 1250) { // Left-Right Control
          digitalWrite(LEFT_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else if (CH1 > 1750) {
          digitalWrite(RIGHT_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else if (CH2 < 1250) { // forward-backward control
          digitalWrite(BACKWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else if (CH2 > 1750) {
          digitalWrite(FORWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(75);
          digitalWrite(SPIKE_PIN, LOW);
          delay(25);
        } else {
          digitalWrite(SPIKE_PIN, LOW);
          digitalWrite(BACKWARD_PIN, LOW);
          digitalWrite(FORWARD_PIN, LOW);
          digitalWrite(RIGHT_PIN, LOW);
          digitalWrite(LEFT_PIN, LOW);
        }
      }
      if (CH6 < 1250) {
        if (CH1 < 1250 && CH2 < 1250) {
          digitalWrite(LEFT_PIN, HIGH);
          digitalWrite(BACKWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN, HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN,LOW);
          delay(50);
        } else if (CH1 < 1250 && CH2 > 1750) {
          digitalWrite(LEFT_PIN, HIGH);
          digitalWrite(FORWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN,LOW);
          delay(50);
        } else if (CH1 > 1750 && CH2 < 1250) {
          digitalWrite(RIGHT_PIN, HIGH);
          digitalWrite(BACKWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN,LOW);
          delay(50);
        } else if (CH1 > 1750 && CH2 > 1750) {
          digitalWrite(RIGHT_PIN, HIGH);
          digitalWrite(FORWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN,LOW);
          delay(50);
        } else if (CH1 < 1250) { // Left-Right Control
          digitalWrite(LEFT_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN,LOW);
          delay(50);
        } else if (CH1 > 1750) {
          digitalWrite(RIGHT_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN,LOW);
          delay(50);
        } else if (CH2 < 1250) { // forward-backward control
          digitalWrite(BACKWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN, LOW);
          delay(50);
        } else if (CH2 > 1750) {
          digitalWrite(FORWARD_PIN, HIGH);
          digitalWrite(SPIKE_PIN,HIGH);
          delay(150);
          digitalWrite(SPIKE_PIN, LOW);
          delay(50);
        } else {
          digitalWrite(BACKWARD_PIN, LOW);
          digitalWrite(FORWARD_PIN, LOW);
          digitalWrite(RIGHT_PIN, LOW);
          digitalWrite(LEFT_PIN, LOW);
          digitalWrite(SPIKE_PIN,LOW);
        }
      }
    }
  }

  // IMU Readings at 50 Hz frequency
  if (currentTime - lastSensorReadTime >= 20) {  // 50 Hz update frequency (20 ms)
    lastSensorReadTime = currentTime;
    if (ppm.get(7) > 1750) {
      if (!Calibrated) {
        Serial.println("Calibrating");
        calibrateSensors();
      } else {
        processSensorData();
      }
    }
  }
}

void calibrateSensors() {
  static int numReadings = 0;

  
  
  if (numReadings <= 1000) {
    sensors_event_t aevent, mevent, gevent;
    accelmag.getEvent(&aevent, &mevent);
    gyro.getEvent(&gevent);

    // Raw sensor readings
    float ax = aevent.acceleration.x;
    float ay = aevent.acceleration.y;
    float az = aevent.acceleration.z;
    float gx = gevent.gyro.x;
    float gy = gevent.gyro.y;
    float gz = gevent.gyro.z;

    // --- Apply Low-Pass Filter ---
    for (int j = 0; j < 3; j++) {
      accelLPF[j] = alpha * (j == 0 ? ax : (j == 1 ? ay : az)) + (1.0 - alpha) * accelLPF[j];
      gyroLPF[j] = alpha * (j == 0 ? gx : (j == 1 ? gy : gz)) + (1.0 - alpha) * gyroLPF[j];
    }

    // --- Apply High-Pass Filter ---
    for (int j = 0; j < 3; j++) {
      float currentAccel = (j == 0 ? ax : (j == 1 ? ay : az));
      float currentGyro = (j == 0 ? gx : (j == 1 ? gy : gz));
      accelHPF[j] = alpha * (accelHPF[j] + currentAccel - accelPrev[j]);
      gyroHPF[j] = alpha * (gyroHPF[j] + currentGyro - gyroPrev[j]);
      accelPrev[j] = currentAccel;
      gyroPrev[j] = currentGyro;
    }

    // --- Apply Madgwick Filter (for orientation correction) ---
    MadgwickFilter(gx, gy, gz, ax, ay, az);
    accelDataMadgwick[0] = ax;
    accelDataMadgwick[1] = ay;
    accelDataMadgwick[2] = az;
    gyroDataMadgwick[0] = gx;
    gyroDataMadgwick[1] = gy;
    gyroDataMadgwick[2] = gz;

    // Sum all three filtered sets (feel free to choose one or mix them as shown)
    for (int j = 0; j < 3; j++) {
      // Example: mix LPF and Madgwick-filtered data
      accelSum[j] += 0.5 * accelLPF[j] + 0.5 * accelDataMadgwick[j];
      gyroSum[j] += 0.5 * gyroLPF[j] + 0.5 * gyroDataMadgwick[j];
    }
    numReadings++;
  }

  if (numReadings > 1000) {
    for (int i = 0; i < 3; i++) {
    accelOffset[i] = accelSum[i] / numReadings;
    gyroOffset[i] = gyroSum[i] / numReadings;
    }
    Calibrated = true;
    Serial.println("Calibrated");
  }
}

void processSensorData() {
  unsigned long currentMillis = millis();
  deltat = (currentMillis - lastUpdate) / 1000.0;
  lastUpdate = currentMillis;

  // Read sensor data
  sensors_event_t aevent, mevent, gevent;
  accelmag.getEvent(&aevent, &mevent);
  gyro.getEvent(&gevent);

  // Apply calibration offsets
  accelData[0] = aevent.acceleration.x - accelOffset[0];
  accelData[1] = aevent.acceleration.y - accelOffset[1];
  accelData[2] = aevent.acceleration.z - accelOffset[2];

  gyroData[0] = gevent.gyro.x - gyroOffset[0];
  gyroData[1] = gevent.gyro.y - gyroOffset[1];
  gyroData[2] = gevent.gyro.z - gyroOffset[2];

  // Apply the Madgwick filter
  MadgwickFilter(gyroData[0], gyroData[1], gyroData[2], accelData[0], accelData[1], accelData[2]);

  // Apply the high-pass filter to the accelerometer data
  applyHighPassFilter();

  // Update filtered accelerometer and gyroscope data
  gyroFiltered[0] = gyroData[0];
  gyroFiltered[1] = gyroData[1];
  gyroFiltered[2] = gyroData[2];

  // Low-pass filtering the accelerometer data
  accelFiltered[0] = alpha * accelFiltered[0] + (1 - alpha) * accelData[0];
  accelFiltered[1] = alpha * accelFiltered[1] + (1 - alpha) * accelData[1];
  accelFiltered[2] = alpha * accelFiltered[2] + (1 - alpha) * accelData[2];

  // Integrating accelerometer data to calculate velocity
  velocity[0] += accelFiltered[0] * deltat;
  velocity[1] += accelFiltered[1] * deltat;
  velocity[2] += accelFiltered[2] * deltat;

  // Calculating impulse momentum
  momentum[0] = velocity[0] * mass;
  momentum[1] = velocity[1] * mass;
  momentum[2] = velocity[2] * mass;

  // Write data to CSV
  File Acceldata = SD.open("Accel.csv", FILE_WRITE);
  if (Acceldata) {
    Acceldata.print(currentMillis);
    Acceldata.print(",");
    Acceldata.print(accelFiltered[0], 4);
    Acceldata.print(",");
    Acceldata.println(accelFiltered[1], 4);
    Acceldata.close();
  } else {
  Serial.println("Error writing to acceleration file.");
  }
  File Velocitydata = SD.open("Velocity.csv", FILE_WRITE);
  if (Velocitydata) {
    Velocitydata.print(currentMillis);
    Velocitydata.print(",");
    Velocitydata.print(velocity[0], 4);
    Velocitydata.print(",");
    Velocitydata.println(velocity[1], 4);
    Velocitydata.close();
  } else {
  Serial.println("Error writing to Velocity file.");
  }
  File Momentumdata = SD.open("Momentum.csv", FILE_WRITE);
  if (Momentumdata) {
    Momentumdata.print(currentMillis);
    Momentumdata.print(",");
    Momentumdata.print(momentum[0], 4);
    Momentumdata.print(",");
    Momentumdata.println(momentum[1], 4);
    Momentumdata.close();
  } else {
  Serial.println("Error writing to momentum file.");
  }

  // Print acceleration and velocity to Serial Monitor
  Serial.print("Accel (m/s^2): ");
  Serial.print("X: "); Serial.print(accelFiltered[0]); 
  Serial.print(", Y: "); Serial.print(accelFiltered[1]); 
  Serial.print(", Z: "); Serial.println(accelFiltered[2]);

  Serial.print("Velocity (m/s): ");
  Serial.print("X: "); Serial.print(velocity[0]); 
  Serial.print(", Y: "); Serial.print(velocity[1]); 
  Serial.print(", Z: "); Serial.println(velocity[2]);
}
