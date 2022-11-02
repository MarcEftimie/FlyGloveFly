#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <MPU6050.h>

// Sensor initialization
MPU6050 mpu;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int FLEX_PIN = A1; // Pin connected to voltage divider output
const int RST_PIN = 12;

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 49000.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 27000.0; // resistance when straight
const float BEND_RESISTANCE = 56000.0; // resistance at 90 deg

// Timers
const float BEND_INTERVAL = 1000;
float bend_timer = millis();

unsigned long GYRO_INTERVAL = 1;
float gyro_timer = millis();
float timeStep = 0.01;

const float RST_BUTTON_INTERVAL = 50;
float rst_button_timer = millis();

unsigned long MOTOR_INTERVAL = 50;
float motor_timer = millis();

// Sensor values
float pitch = 0;
float roll = 0;
float yaw = 0;

int bend_angle = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);
  pinMode(RST_PIN, INPUT);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop() 
{

  if (millis() - gyro_timer > (timeStep * 1000)) {
    // Read normalized values
    Vector norm = mpu.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;

    // Output raw
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);  
    Serial.print(" Yaw = ");
    Serial.println(yaw);

    gyro_timer = millis();
  }

  if (millis() - rst_button_timer > RST_BUTTON_INTERVAL) {
    if (digitalRead(RST_PIN) == 0) {
      Serial.println("RST");
      pitch = 0;
      roll = 0;
      yaw = 0;
    }
    rst_button_timer = millis();
  }

  if (millis() - bend_timer > BEND_INTERVAL) {
      int flexADC = analogRead(FLEX_PIN);
      float flexV = flexADC * VCC / 1023.0;
      float flexR = R_DIV * (VCC / flexV - 1.0);
      Serial.println("Resistance: " + String(flexR) + " ohms");
    
      bend_angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                       0, 90.0);
      Serial.println("Bend: " + String(bend_angle) + " degrees");
      Serial.println();

      if (10 <= bend_angle && bend_angle < 25) {
        Serial.println("10-25");
      } else if (25 < bend_angle && bend_angle <= 40) {
        Serial.println("25-40");
      } else if (40 < bend_angle && bend_angle <= 55) {
        Serial.println("40-55");
      } else if (55 < bend_angle && bend_angle <= 70) {
        Serial.println("55-70");
      } else {
        Serial.println("Deadzone");
      }

      bend_timer = millis();
  }

  if (millis() - motor_timer > MOTOR_INTERVAL) {

    motor_timer = millis();
  }
  
}
