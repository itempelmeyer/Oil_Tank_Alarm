#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LCD initialization (I2C address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin definitions
int pinRX = 10;        // Ultrasonic sensor RX pin
int pinTX = 11;        // Ultrasonic sensor TX pin
int Relay1 = 7;        // Relay 1 pin
int Relay2 = 6;        // Relay 2 pin

// Distance thresholds in millimeters for relay control
int Critical = 25;    // Critical threshold 2"
int Danger = 50;      // Danger threshold 3"
int Warning = 75;     // Warning threshold 4"

// Analog voltage sensing configuration
const int analogPin = A0;      // Analog pin for voltage divider
const float R1 = 50020.0;      // Resistor R1 value in ohms
const float R2 = 10050.0;      // Resistor R2 value in ohms
const float referenceVoltage = 5.0; // Arduino reference voltage (typically 5V)

// Battery voltage-to-charge mapping table
const float voltageTable[][2] = {
    {16.0, 0.0}, {17.0, 12.5}, {17.5, 25.0}, {18.0, 37.5},
    {18.5, 50.0}, {19.0, 62.5}, {19.5, 75.0}, {20.0, 87.5},
    {21.0, 100.0} // End of table
};
const int tableSize = sizeof(voltageTable) / sizeof(voltageTable[0]);

// Startup delay to prevent alarms during initialization
const unsigned long startupDelay = 5000; // 3 seconds
unsigned long startupTime = 0;           // Record startup time

// Ultrasonic sensor data processing
unsigned char data_buffer[4] = {0}; // Buffer for raw sensor data
int Clearance = 0;                  // Measured clearance in mm
unsigned char CS;                   // Checksum for validation

// Rolling average variables for clearance smoothing
const int rollingSamples = 5;       // Number of samples for averaging
int clearanceSamples[rollingSamples] = {0}; // Circular buffer for samples
int sampleIndex = 0;                // Current index in circular buffer
int averageClearance = 0;           // Smoothed clearance
const float alpha = 0.5;            // Smoothing factor
float smoothedClearance = 0.0;      // Exponentially smoothed clearance

// Voltage smoothing variables
const float voltageAlpha = 0.07;     // Smoothing factor for voltage
float smoothedVoltage = 0.0;        // Smoothed voltage

// Ultrasonic sensor communication
SoftwareSerial mySerial(pinRX, pinTX); // Software serial for ultrasonic sensor

// Make custom characters:
byte OR[] = {
  B01010,
  B01010,
  B01010,
  B01010,
  B01010,
  B01010,
  B01010,
  B01010
};

void setup() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, OR);

  // Initialize serial communication
  Serial.begin(9600);
  mySerial.begin(9600);

  // Configure relay pins as outputs
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  digitalWrite(Relay1, HIGH); // Relays off
  digitalWrite(Relay2, HIGH); // Relays off

  // Record startup time
  startupTime = millis();
}

// Function to display data on the LCD
void updateDisplay(float clearanceMM, float clearanceInches, float voltage, float chargePercentage) {
  // Display battery voltage
  lcd.setCursor(0, 0);
  lcd.print("Batt:");
  lcd.setCursor(5, 0);
  lcd.print("           "); // Clear old value
  lcd.setCursor(5, 0);
  lcd.print(voltage, 1);
  lcd.setCursor(9, 0);
  lcd.print("V");

  // Display charge percentage
  lcd.setCursor(11, 0);
  lcd.write(byte(0));
  lcd.setCursor(13, 0);
  lcd.print(chargePercentage, 0);
  lcd.setCursor(15, 0);
  lcd.print("%");

  // Display clearance in inches
  lcd.setCursor(0, 1);
  lcd.print("Clearance:");
  lcd.setCursor(10, 1);
  lcd.print("      "); // Clear old value
  lcd.setCursor(10, 1);
  lcd.print(clearanceInches, 1);
  lcd.setCursor(14, 1);
  lcd.print("in");
}

// Function to interpolate charge percentage from voltage
float getChargePercentage(float voltage) {
  for (int i = 0; i < tableSize - 1; i++) {
    if (voltage >= voltageTable[i][0] && voltage <= voltageTable[i + 1][0]) {
      // Linear interpolation
      float v1 = voltageTable[i][0];
      float p1 = voltageTable[i][1];
      float v2 = voltageTable[i + 1][0];
      float p2 = voltageTable[i + 1][1];
      return p1 + (p2 - p1) * ((voltage - v1) / (v2 - v1));
    }
  }
  // Out of range: return 0% or 100%
  return (voltage < voltageTable[0][0]) ? 0 : 100;
}

void loop() {
  // Analog voltage processing
  int analogValue = analogRead(analogPin); // Read analog value
  float voltage = (analogValue / 1023.0) * referenceVoltage; // Calculate voltage
  float actualVoltage = voltage * ((R1 + R2) / R2); // Scale to battery voltage
  smoothedVoltage = voltageAlpha * actualVoltage + (1 - voltageAlpha) * smoothedVoltage; // Smooth voltage
  float chargePercentage = getChargePercentage(smoothedVoltage); // Get charge percentage

  // Trigger ultrasonic sensor
  digitalWrite(pinTX, HIGH);
  delay(10);
  digitalWrite(pinTX, LOW);

  // Process ultrasonic sensor data
  if (mySerial.available() > 0) {
    delay(4);
    if (mySerial.read() == 0xff) { // Packet header
      data_buffer[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = mySerial.read();
      }

      // Validate checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      if (data_buffer[3] == CS) {
        Clearance = (data_buffer[1] << 8) + data_buffer[2];

        // Update rolling average
        clearanceSamples[sampleIndex] = Clearance;
        sampleIndex = (sampleIndex + 1) % rollingSamples;
        int sum = 0;
        for (int i = 0; i < rollingSamples; i++) {
          sum += clearanceSamples[i];
        }
        averageClearance = sum / rollingSamples;

        // Exponential smoothing
        smoothedClearance = alpha * averageClearance + (1 - alpha) * smoothedClearance;
      }
    }
  }

  unsigned long elapsedTime = millis() - startupTime;

  if (elapsedTime > startupDelay) {
    // Relay control logic
    if (averageClearance >= Warning) {
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, HIGH);
      Serial.print("Stage 3; ");
    } else if (averageClearance < Warning && averageClearance >= Danger) {
      digitalWrite(Relay1, LOW);
      digitalWrite(Relay2, HIGH);
      Serial.print("Stage 2; ");
    } else if (averageClearance < Danger && averageClearance >= Critical) {
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, LOW);
      Serial.print("Stage 1; ");
    } else if (averageClearance < Critical) {
      digitalWrite(Relay1, LOW);
      digitalWrite(Relay2, LOW);
      Serial.print("Stage 0; ");
    }

    // Convert clearance to inches
    float ClearanceInches = smoothedClearance / 25.4;

    // Serial monitor output
    Serial.print("Voltage (Raw): ");
    Serial.print(actualVoltage, 2);
    Serial.print(" V, Voltage (Smoothed): ");
    Serial.print(smoothedVoltage, 2);
    Serial.print(" V, Charge: ");
    Serial.print(chargePercentage, 2);
    Serial.print(" %, Clearance: ");
    Serial.print(averageClearance);
    Serial.println(" mm");

    // Update LCD display
    updateDisplay(smoothedClearance, ClearanceInches, smoothedVoltage, chargePercentage);
  } else {
    // During startup delay
    digitalWrite(Relay1, HIGH);
    digitalWrite(Relay2, HIGH);
    Serial.println("Startup delay active, relays off");

    lcd.setCursor(0, 0);
    lcd.print("Startup Delay");
    lcd.setCursor(0, 1);
    lcd.print("      "); // Clear old value
    lcd.setCursor(0, 1);
    lcd.print(startupDelay - elapsedTime);
    lcd.setCursor(4, 1);
    lcd.print("ms");
  }

  delay(20); // Prevent rapid updates
}
