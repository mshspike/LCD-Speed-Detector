#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define pins for ultrasonic sensors
const int TRIG_PIN_1 = 2;  // First sensor trigger pin
const int ECHO_PIN_1 = 3;  // First sensor echo pin
const int TRIG_PIN_2 = 4;  // Second sensor trigger pin
const int ECHO_PIN_2 = 5;  // Second sensor echo pin
const int BUZZER_PIN = 6;  // Buzzer pin

// Constants
const float MIN_DISTANCE = 5.0;      // Minimum detection distance in cm
const float MAX_DISTANCE = 70.0;     // Maximum detection distance in cm
const float SOUND_SPEED = 0.034;     // Speed of sound in cm/microsecond
const unsigned long TRIGGER_TIMEOUT = 5000;  // Timeout for second trigger in milliseconds
const int BUZZER_FREQUENCY = 2000;   // Buzzer frequency in Hz
const int BUZZER_DURATION = 200;     // Buzzer duration in milliseconds
const float SENSOR_DISTANCE = 70.0;  // Distance between sensors in cm
const unsigned long BACKLIGHT_TIMEOUT = 30000;  // Backlight timeout in milliseconds (30 seconds)
const unsigned long SENSOR_READ_INTERVAL = 200;  // Time between sensor readings in milliseconds

// Variables
bool firstSensorTriggered = false;
bool secondSensorTriggered = false;
unsigned long firstTriggerTime = 0;  // Time when first sensor triggered
int firstTriggeredSensor = 0;        // 1 for sensor 1, 2 for sensor 2
float speed = 0;                     // Speed in km/h
unsigned long lastActivityTime = 0;  // Time of last activity
unsigned long lastSensorRead = 0;    // Time of last sensor reading

// Initialize LCD (address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Speed Detector Debug Mode");
  Serial.println("------------------------");
  Serial.println("Sensor 1 | Sensor 2 | Status");
  Serial.println("------------------------");
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);  // Initialize buzzer pin
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed Detector");
  lcd.setCursor(0, 1);
  lcd.print("Ready...");
  
  lastActivityTime = millis();  // Initialize activity timer
  lastSensorRead = millis();    // Initialize sensor read timer
}

void beep() {
  tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
}

float getDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10us pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance
  float distance = duration * SOUND_SPEED / 2;
  return distance;
}

void printDebugInfo(float dist1, float dist2, const char* status) {
  // Print sensor readings and status
  Serial.print(dist1);
  Serial.print(" cm\t| ");
  Serial.print(dist2);
  Serial.print(" cm\t| ");
  Serial.println(status);
}

void resetDetection() {
  firstSensorTriggered = false;
  secondSensorTriggered = false;
  firstTriggerTime = 0;
  firstTriggeredSensor = 0;
  speed = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed Detector");
  lcd.setCursor(0, 1);
  lcd.print("Ready...");
}

void checkBacklight() {
  unsigned long currentTime = millis();
  if (currentTime - lastActivityTime > BACKLIGHT_TIMEOUT) {
    lcd.noBacklight();  // Turn off backlight
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // Only read sensors at specified intervals to save power
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentTime;
    
    // Read distances from both sensors
    float distance1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    float distance2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    
    bool sensor1InRange = (distance1 >= MIN_DISTANCE && distance1 <= MAX_DISTANCE);
    bool sensor2InRange = (distance2 >= MIN_DISTANCE && distance2 <= MAX_DISTANCE);
    
    // First sensor trigger (either sensor 1 or 2)
    if (!firstSensorTriggered && !secondSensorTriggered) {
      if (sensor1InRange) {
        firstSensorTriggered = true;
        firstTriggerTime = currentTime;
        firstTriggeredSensor = 1;
        lcd.backlight();  // Turn on backlight
        lastActivityTime = currentTime;  // Reset activity timer
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Sensor 1");
        lcd.setCursor(0, 1);
        lcd.print("triggered!");
        printDebugInfo(distance1, distance2, "Sensor 1 triggered");
      }
      else if (sensor2InRange) {
        firstSensorTriggered = true;
        firstTriggerTime = currentTime;
        firstTriggeredSensor = 2;
        lcd.backlight();  // Turn on backlight
        lastActivityTime = currentTime;  // Reset activity timer
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Sensor 2");
        lcd.setCursor(0, 1);
        lcd.print("triggered!");
        printDebugInfo(distance1, distance2, "Sensor 2 triggered");
      }
    }
    
    // Second sensor trigger
    if (firstSensorTriggered && !secondSensorTriggered) {
      if (currentTime - firstTriggerTime <= TRIGGER_TIMEOUT) {
        // Check for second sensor trigger based on which sensor triggered first
        if ((firstTriggeredSensor == 1 && sensor2InRange) || 
            (firstTriggeredSensor == 2 && sensor1InRange)) {
          secondSensorTriggered = true;
          
          // Calculate speed (km/h)
          unsigned long timeDiff = currentTime - firstTriggerTime;
          // Convert cm/s to km/h: (cm/s * 3600) / 100000
          speed = (SENSOR_DISTANCE * 3600.0) / (timeDiff * 100.0);
          
          // Display speed
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Speed:");
          lcd.setCursor(0, 1);
          lcd.print(speed);
          lcd.print(" km/h");
          
          // Print debug info
          char speedMsg[50];
          sprintf(speedMsg, "Speed: %.2f km/h", speed);
          printDebugInfo(distance1, distance2, speedMsg);
          
          beep();  // Beep once for detection
          delay(2000);
          resetDetection();
        }
      }
    }
    
    // Check for timeout
    if (firstSensorTriggered && !secondSensorTriggered) {
      if (currentTime - firstTriggerTime > TRIGGER_TIMEOUT) {
        printDebugInfo(distance1, distance2, "Timeout - No second trigger");
        resetDetection();
      }
    }
    
    // Print regular status if nothing special happened
    if (!firstSensorTriggered && !secondSensorTriggered) {
      printDebugInfo(distance1, distance2, "Waiting for detection");
    }
  }
  
  // Check backlight timeout
  checkBacklight();
}
