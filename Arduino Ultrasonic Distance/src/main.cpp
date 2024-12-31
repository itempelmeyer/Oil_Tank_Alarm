#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "images.h"

// Function declarations
void initializeLCD();
void initializeSerial();
void initializeRelays();
void displayStartupDelay(unsigned long elapsedTime);
void updateVoltage();
void triggerUltrasonicSensor();
void processUltrasonicData();
void updateRelays();
void outputToSerialMonitor();
void delayForNextUpdate();
float getChargePercentage(float voltage);
void updateDisplay(float clearanceMM, float clearanceInches, float voltage, float chargePercentage);
void displayUptime(unsigned long elapsedTime);

void setup() {
  initializeLCD();
  initializeSerial();
  initializeRelays();

  // Record startup time
  startupTime = millis();
}

void loop() {
  unsigned long elapsedTime = millis() - startupTime;

  updateVoltage();            // Handle voltage and charge percentage calculations
  triggerUltrasonicSensor();  // Trigger the ultrasonic sensor
  processUltrasonicData();    // Read and process sensor data

  if (elapsedTime <= startupDelay) {
    displayStartupDelay(elapsedTime);
  } else {
    updateRelays();           // Update relays based on clearance
    outputToSerialMonitor();  // Send data to Serial Monitor
  }

  delayForNextUpdate();
}

// Function definitions

void initializeLCD() {
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, OR);
}

void initializeSerial() {
  Serial.begin(115200);
  mySerial.begin(9600);
}

void initializeRelays() {
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
}

void displayStartupDelay(unsigned long elapsedTime) {
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  displayUptime(elapsedTime);
  Serial.println("Startup delay active, relays off");

  lcd.setCursor(0, 0);
  lcd.print("Startup Delay");

  lcd.setCursor(0, 1);
  lcd.print("Wait ");
  lcd.print((startupDelay - elapsedTime) / 1000 + 1);
  lcd.print("s   ");
}

void updateVoltage() {
  int analogValue = analogRead(analogPin);
  float voltage = (analogValue / 1023.0) * referenceVoltage;
  actualVoltage = voltage * ((R1 + R2) / R2);
  smoothedVoltage = voltageAlpha * actualVoltage + (1 - voltageAlpha) * smoothedVoltage;
}

void triggerUltrasonicSensor() {
  digitalWrite(pinTX, HIGH);
  delay(10);
  digitalWrite(pinTX, LOW);
}

void processUltrasonicData() {
  if (mySerial.available() > 0) {
    delay(4);
    if (mySerial.read() == 0xff) {
      data_buffer[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = mySerial.read();
      }

      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      if (data_buffer[3] == CS) {
        rawClearance = (data_buffer[1] << 8) + data_buffer[2];

        clearanceSamples[sampleIndex] = rawClearance;
        sampleIndex = (sampleIndex + 1) % rollingSamples;
        int sum = 0;
        for (int i = 0; i < rollingSamples; i++) {
          sum += clearanceSamples[i];
        }
        averageClearance = sum / rollingSamples;
        smoothedClearance = alpha * averageClearance + (1 - alpha) * smoothedClearance;
      }
    }
  }
}

void updateRelays() {
    unsigned long currentTime = millis();

    // Check if averageClearance < Critical
    if (averageClearance < Critical) {
        if (belowCriticalStartTime == 0) {
            // Start the timer if it's the first time below Critical
            belowCriticalStartTime = currentTime;
        } else if (currentTime - belowCriticalStartTime >= 2500 && !state5Latched) {
            // Latch State 5 if below Critical for 3 seconds
            state5Latched = true;
            state5LatchTime = currentTime;
            Serial.println("State 5 Latched");
        }
    } else {
        // Reset the timer if clearance is not below Critical
        belowCriticalStartTime = 0;
    }

    // Handle latched State 5
    if (state5Latched) {
        if (currentTime - state5LatchTime < 60000) {
            // Remain in State 5 for 21 seconds
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, LOW);
            Stage = 5;
            return; // Prevent transitions to other states
        } else {
            // Allow other state transitions after 21 seconds
            state5Latched = false;
            belowCriticalStartTime = 0; // Reset belowCritical tracking
            Serial.println("State 5 Unlatched");
        }
    }

    // Normal state transitions
    if (averageClearance >= Warning) {
        digitalWrite(Relay1, HIGH);
        digitalWrite(Relay2, HIGH);
        Stage = 1;
    } else if (averageClearance < Warning && averageClearance >= Danger) {
        digitalWrite(Relay1, LOW);
        digitalWrite(Relay2, HIGH);
        Stage = 2;
    } else if (averageClearance < Danger && averageClearance >= Critical) {
        digitalWrite(Relay1, HIGH);
        digitalWrite(Relay2, LOW);
        Stage = 3;
    } else if (averageClearance < Critical) {
        digitalWrite(Relay1, LOW);
        digitalWrite(Relay2, LOW);
        Stage = 4;
    }
}


void outputToSerialMonitor() {
  unsigned long elapsedTime = millis() - startupTime;
  displayUptime(elapsedTime); // Display uptime in serial monitor
  Serial.print("rawClearance: " + String(rawClearance) + "mm, ");
  Serial.print("averageClearance: " + String(averageClearance) + "mm, ");
  Serial.print("SmoothedClearance: " + String(smoothedClearance) + "mm, ");
  Serial.print("Stage: " + String(Stage) + ", ");
  float clearanceInches = smoothedClearance / 25.4;
  Serial.print("actualVoltage: " + String(actualVoltage, 2) + "V, ");
  Serial.print("smoothedVoltage: " + String(smoothedVoltage, 2) + "V, ");
  Serial.println("Charge: " + String(getChargePercentage(smoothedVoltage), 2) + "%");

  updateDisplay(averageClearance, clearanceInches, smoothedVoltage, getChargePercentage(smoothedVoltage));
}

void displayUptime(unsigned long elapsedTime) {
  // Calculate uptime in seconds
  float uptimeSeconds = elapsedTime / 1000.0;

  // Print to serial monitor
  Serial.print("Uptime: " + String(uptimeSeconds, 2) + "s, ");  
}

void delayForNextUpdate() {
  delay(0);
}

float getChargePercentage(float voltage) {
  for (int i = 0; i < tableSize - 1; i++) {
    if (voltage >= voltageTable[i][0] && voltage <= voltageTable[i + 1][0]) {
      float v1 = voltageTable[i][0];
      float p1 = voltageTable[i][1];
      float v2 = voltageTable[i + 1][0];
      float p2 = voltageTable[i + 1][1];
      return p1 + (p2 - p1) * ((voltage - v1) / (v2 - v1));
    }
  }
  return (voltage < voltageTable[0][0]) ? 0 : 100;
}

void updateDisplay(float clearanceMM, float clearanceInches, float voltage, float chargePercentage) {
    // LCD display logic
    lcd.setCursor(0, 0);
    lcd.print("Batt:");
    lcd.setCursor(5, 0);
    lcd.print("           ");
    lcd.setCursor(5, 0);
    lcd.print(voltage, 1);
    lcd.setCursor(9, 0);
    lcd.print("V");

    lcd.setCursor(11, 0);
    lcd.write(byte(0));
    lcd.setCursor(13, 0);
    lcd.print(chargePercentage, 0);
    lcd.setCursor(15, 0);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Clearance:");
    lcd.setCursor(10, 1);
    lcd.print("      ");
    lcd.setCursor(10, 1);
    lcd.print(clearanceInches, 1);
    lcd.setCursor(14, 1);
    lcd.print("in");
}
