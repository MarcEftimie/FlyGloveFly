/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/
const int FLEX_PIN = A0; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 49000.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 27000.0; // resistance when straight
const float BEND_RESISTANCE = 56000.0; // resistance at 90 deg

const float BEND_INTERVAL = 1000;
float bend_current_time = millis();

int angle = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);
}

void loop() 
{

  if (millis() - bend_current_time > BEND_INTERVAL) {
      int flexADC = analogRead(FLEX_PIN);
      float flexV = flexADC * VCC / 1023.0;
      float flexR = R_DIV * (VCC / flexV - 1.0);
      Serial.println("Resistance: " + String(flexR) + " ohms");
    
      angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                       0, 90.0);
      Serial.println("Bend: " + String(angle) + " degrees");
      Serial.println();

      if (10 <= angle && angle < 25) {
        Serial.println("10-25");
      } else if (25 < angle && angle <= 40) {
        Serial.println("25-40");
      } else if (40 < angle && angle <= 55) {
        Serial.println("40-55");
      } else if (55 < angle && angle <= 70) {
        Serial.println("55-70");
      } else {
        Serial.println("Deadzone");
      }
      bend_current_time = millis();
  }
  
}
