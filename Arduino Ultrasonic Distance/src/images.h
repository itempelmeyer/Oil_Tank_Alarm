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
int rawClearance = 0;                  // Measured clearance in mm
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
float actualVoltage = 0; // Global variable to store the actual voltage


// LCD initialization (I2C address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

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

float clearanceMM;
int Stage = 0;