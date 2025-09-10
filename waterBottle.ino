// Smart Water Reminder & Usage Monitor
#include "thingProperties.h"
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

// Component Configuration
#define TRIG_PIN          12
#define ECHO_PIN          14
#define LED_PIN           2
#define BUZZER_PIN        13
#define TOUCH_PIN         4
#define POTENTIOMETER_PIN 34
#define SERVO_PIN         27

// Operational Constants
#define BOTTLE_HEIGHT           20    // cm
#define BOTTLE_ABSENT_THRESHOLD 19    // cm
#define TOUCH_THRESHOLD         40
#define REMINDER_MIN            30    // minutes
#define REMINDER_MAX            90    // minutes

// Global Objects
BluetoothSerial SerialBT;
Servo capServo;

// System State
struct {
  uint8_t waterLevel = 0;
  uint16_t totalDrinks = 0;
  bool bottlePresent = false;
  bool reminderActive = false;
  unsigned long lastDrinkTime = 0;
  unsigned long lastReminderTime = 0;
} SystemState;

void setupComponents() {
  // Initialize sensors and actuators
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  capServo.attach(SERVO_PIN);
  capServo.write(0);
}

void setupConnectivity() {
  if (!SerialBT.begin("WaterReminderESP32")) {
    Serial.println("Bluetooth initialization failed!");
  }
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  setupComponents();
  setupConnectivity();
  initProperties();

  // Initial alert
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 1000, 200);
  delay(300);
  digitalWrite(LED_PIN, LOW);
}

void measureWaterLevel() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const long duration = pulseIn(ECHO_PIN, HIGH);
  const float distance = duration * 0.034 / 2;

  SystemState.bottlePresent = (distance < BOTTLE_ABSENT_THRESHOLD);

  if (SystemState.bottlePresent) {
    const int level = constrain(BOTTLE_HEIGHT - distance, 0, BOTTLE_HEIGHT);
    SystemState.waterLevel = map(level, 0, BOTTLE_HEIGHT, 0, 100);

    if (SystemState.waterLevel < 15 && !SystemState.reminderActive) {
      alertUser(3);
    }
  } else {
    SystemState.waterLevel = 0;
  }

  // Send value to Cloud (make sure "waterLevelCloud" exists in thingProperties.h)
  waterLevelCloud = SystemState.waterLevel;
  totalDrinksCloud = SystemState.totalDrinks;
}

void handleUserInput() {
  static unsigned long lastTouchTime = 0;
  const int touchValue = touchRead(TOUCH_PIN);

  if (touchValue < TOUCH_THRESHOLD && (millis() - lastTouchTime) > 200) {
    lastTouchTime = millis();
    SystemState.lastDrinkTime = millis();
    SystemState.totalDrinks++;

    // Reset reminder when drinking
    SystemState.reminderActive = false;
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);

    // Simulate bottle cap open/close
    capServo.write(90);
    delay(500);
    capServo.write(0);

    // Update cloud drink count
    totalDrinksCloud = SystemState.totalDrinks;
  }
}

void updateReminderSystem() {
  const int potValue = analogRead(POTENTIOMETER_PIN);
  const uint8_t interval = map(potValue, 0, 4095, REMINDER_MIN, REMINDER_MAX);
  const unsigned long intervalMs = interval * 60000UL;

  if (SystemState.bottlePresent &&
      (millis() - SystemState.lastDrinkTime) > intervalMs &&
      !SystemState.reminderActive) {
    activateReminder();
  }

  if (SystemState.reminderActive) {
    const unsigned long currentMillis = millis();
    if (currentMillis - SystemState.lastReminderTime > 500) {
      SystemState.lastReminderTime = currentMillis;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));

      if (digitalRead(LED_PIN)) {
        tone(BUZZER_PIN, 2000, 100);
      }
    }
  }
}

void loop() {
  ArduinoCloud.update();

  static unsigned long lastMeasurement;
  if (millis() - lastMeasurement > 2000) {
    measureWaterLevel();
    lastMeasurement = millis();
  }

  handleUserInput();
  updateReminderSystem();
}

// Helper functions
void activateReminder() {
  SystemState.reminderActive = true;
  SystemState.lastReminderTime = millis();
  alertUser(2);
}

void alertUser(uint8_t beepCount) {
  for (uint8_t i = 0; i < beepCount; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000, 200);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}

// Cloud callback
void onReminderIntervalChange() {
  reminderInterval = constrain(reminderInterval, 15, 120);
}
