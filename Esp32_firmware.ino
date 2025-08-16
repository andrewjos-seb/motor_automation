#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <ArduinoOTA.h>
#include "esp_task_wdt.h"
#include <Firebase_ESP_Client.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// --- Pin Definitions ---
#define MOTOR_PIN 23
#define TOP_SENSOR_PIN 22

// --- WiFi and Firebase Credentials ---
#define WIFI_SSID "FTTH-bsnl"
#define WIFI_PASSWORD "adavichira@1"
#define API_KEY "AIzaSyCdQfrxQojddcJtMMkFGePzlbQjBlsL7AY"
#define DATABASE_URL "kapithan-ff7c3-default-rtdb.firebaseio.com"
#define USER_EMAIL "andrewjos2004@gmail.com"
#define USER_PASSWORD "andrew@1"

// --- Timing and Logic Constants ---
const unsigned long MOTOR_MAX_RUN_TIME_MS = 12 * 60 * 1000UL;
const unsigned long MOTOR_COOLDOWN_PERIOD_MS = 3 * 3600 * 1000UL;
const unsigned long FIREBASE_CHECK_INTERVAL_MS = 2500;  // Increased slightly for stability

// --- System State Machine ---
enum SystemState {
  STATE_IDLE,
  STATE_WAITING_FOR_COOLDOWN,
  STATE_MOTOR_RUNNING,
  STATE_TANK_FULL
};
SystemState currentState = STATE_IDLE;

// *** NEW: Variables for state synchronization and logging queue
SystemState lastReportedState = (SystemState)-1;  // -1 forces an initial update
enum PendingLogEvent { LOG_NONE,
                       LOG_MOTOR_ON,
                       LOG_MOTOR_OFF };
PendingLogEvent pendingLog = LOG_NONE;

// --- Firebase Objects ---
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// --- Global Variables for State Management ---
unsigned long lastMotorRunStopTime = 0;
unsigned long motorStartTime = 0;
unsigned long lastFirebaseCheckTime = 0;
bool manualOverride = false;

// =======================================================================
// SETUP
// =======================================================================
bool debounceSensor(int pin, int stableReadCount = 5, int delayMs = 10) {
  bool lastState = digitalRead(pin);
  for (int i = 0; i < stableReadCount; i++) {
    delay(delayMs);
    bool currentState = digitalRead(pin);
    if (currentState != lastState) {
      return debounceSensor(pin, stableReadCount, delayMs);
    }
  }
  return lastState;
}

void setupFirebase() {
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

void setup() {
  Serial.begin(115200);

  esp_task_wdt_config_t wdt_config = { .timeout_ms = 20000, .idle_core_mask = 1, .trigger_panic = true };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(TOP_SENSOR_PIN, INPUT);

  connectToWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    syncTime();
    setupFirebase();
    setupOTA();
  } else {
    Serial.println("Failed to connect to Wi-Fi. Operating in limited offline mode.");
  }

  if (debounceSensor(TOP_SENSOR_PIN) == HIGH) {
    currentState = STATE_TANK_FULL;
    Serial.println("Initial State: TANK_FULL");
  } else {
    currentState = STATE_WAITING_FOR_COOLDOWN;
    Serial.println("Initial State: COOLDOWN");
  }
}

// =======================================================================
// MAIN LOOP (State Machine)
// =======================================================================
void loop() {
  esp_task_wdt_reset();
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // *** MODIFIED: This block now handles ALL Firebase interactions ***
  if (Firebase.ready() && millis() - lastFirebaseCheckTime > FIREBASE_CHECK_INTERVAL_MS) {
    lastFirebaseCheckTime = millis();
    checkFirebaseCommand();  // This READS from Firebase
    syncFirebaseState();     // This WRITES and LOGS to Firebase
  }

  // --- Main State Machine Logic (This runs fast and freely) ---
  switch (currentState) {
    case STATE_IDLE:
      motor_start();
      break;
    case STATE_WAITING_FOR_COOLDOWN:
      if (millis() - lastMotorRunStopTime >= MOTOR_COOLDOWN_PERIOD_MS) {
        Serial.println("Motor cooldown finished. Returning to IDLE state.");
        currentState = STATE_IDLE;
      }
      break;
    case STATE_MOTOR_RUNNING:
      if (debounceSensor(TOP_SENSOR_PIN) == HIGH) {
        Serial.println("Tank is now full (detected by sensor). Stopping motor.");
        motor_stop();
        currentState = STATE_TANK_FULL;
      } else if (millis() - motorStartTime >= MOTOR_MAX_RUN_TIME_MS) {
        Serial.println("Motor has reached max run time. Stopping motor (timeout).");
        motor_stop();
        currentState = STATE_WAITING_FOR_COOLDOWN;
      }
      break;
    case STATE_TANK_FULL:
      if (debounceSensor(TOP_SENSOR_PIN) == LOW) {
        Serial.println("Water level dropped. Moving to cooldown state.");
        currentState = STATE_WAITING_FOR_COOLDOWN;
      }
      break;
  }
}

// =======================================================================
// HELPER FUNCTIONS
// =======================================================================


// *** MODIFIED: Now only queues a log event, does NOT call Firebase
void motor_start() {
  if (manualOverride) return;

  Serial.println("Starting motor...");
  digitalWrite(MOTOR_PIN, HIGH);
  motorStartTime = millis();
  currentState = STATE_MOTOR_RUNNING;
  pendingLog = LOG_MOTOR_ON;  // Queue the log event
}

// *** MODIFIED: Now only queues a log event, does NOT call Firebase
void motor_stop() {
  Serial.println("Stopping motor...");
  digitalWrite(MOTOR_PIN, LOW);
  lastMotorRunStopTime = millis();
  pendingLog = LOG_MOTOR_OFF;  // Queue the log event
}

// --- THIS IS THE NEW CORE FUNCTION FOR ALL FIREBASE WRITES ---
void syncFirebaseState() {
  // 1. Process any pending log events
  if (pendingLog == LOG_MOTOR_ON) {
    logMotorEvent("ON");
  } else if (pendingLog == LOG_MOTOR_OFF) {
    logMotorEvent("OFF");
  }
  pendingLog = LOG_NONE;  // Clear the queue after processing

  // 2. Sync the current state if it has changed since last report
  if (currentState != lastReportedState) {
    Serial.printf("State changed from %d to %d. Syncing with Firebase.\n", lastReportedState, currentState);
    switch (currentState) {
      case STATE_IDLE: break;
      case STATE_WAITING_FOR_COOLDOWN:
        updateFirebaseStatus("/Auto_Tank/D1_motor", "0");
        updateFirebaseStatus("/Auto_Tank/D1_update", "Tank_Empty");
        break;
      case STATE_MOTOR_RUNNING:
        updateFirebaseStatus("/Auto_Tank/D1_motor", "1");
        // No change to tank level status while motor is running
        break;
      case STATE_TANK_FULL:
        updateFirebaseStatus("/Auto_Tank/D1_motor", "0");
        updateFirebaseStatus("/Auto_Tank/D1_update", "Tank_Full");
        // Also update the lastFull time when this state is confirmed
        time_t now = time(nullptr);
        if (now > 10000) Firebase.RTDB.set(&fbdo, "/Auto_Tank/lastFULL", formatTimestamp(now));
        break;
    }
    lastReportedState = currentState;  // Mark the state as reported
  }
}

// *** UNCHANGED: This function is now only called by syncFirebaseState ***
void logMotorEvent(const char* eventType) {
  time_t current_timestamp = time(nullptr);
  if (current_timestamp < 10000) {
    Serial.println("Time not synced, cannot log event.");
    return;
  }

  String humanReadableTime = formatTimestamp(current_timestamp);
  String logPath = "/Auto_Tank/Log/" + String(current_timestamp);

  Firebase.RTDB.set(&fbdo, logPath + "/event", eventType);
  Firebase.RTDB.set(&fbdo, logPath + "/time", humanReadableTime);

  if (strcmp(eventType, "ON") == 0) {
    Firebase.RTDB.set(&fbdo, "/Auto_Tank/lastON", humanReadableTime);
  } else if (strcmp(eventType, "OFF") == 0) {
    Firebase.RTDB.set(&fbdo, "/Auto_Tank/lastOFF", humanReadableTime);
  }
  Serial.printf("Logged motor event '%s' to Firebase.\n", eventType);
}

// *** UNCHANGED: This is a write operation, but it's okay to leave as is for now
// as it's part of a read-modify-write cycle on a single command path.
void checkFirebaseCommand() {
  if (Firebase.RTDB.get(&fbdo, "/Auto_Tank/D1")) {
    String cmd = fbdo.stringData();
    if (cmd == "1") {
      Serial.println("Manual START command received.");
      manualOverride = false;
      lastMotorRunStopTime = 0;
      currentState = STATE_IDLE;
      updateFirebaseStatus("/Auto_Tank/D1", "3");  // Acknowledge command
    } else if (cmd == "0") {
      Serial.println("Manual STOP command received.");
      manualOverride = true;
      if (currentState == STATE_MOTOR_RUNNING) motor_stop();
      currentState = STATE_WAITING_FOR_COOLDOWN;
      updateFirebaseStatus("/Auto_Tank/D1", "3");  // Acknowledge command
    } else if (cmd == "4") {
      Serial.println("Force Firebase Sync command received.");
      manualOverride = false;  

      // By setting lastReportedState to an invalid value, we force the
      // syncFirebaseState() function to send a full update on its next run.
      lastReportedState = (SystemState)-1;

      // Acknowledge the command was processed.
      updateFirebaseStatus("/Auto_Tank/D1", "ok");
    }
  }
}

// *** UNCHANGED: This function is now the single point of writing for status
void updateFirebaseStatus(const char* path, const char* value) {
  if (Firebase.ready()) {
    Firebase.RTDB.set(&fbdo, path, value);
    Serial.printf("Updated Firebase path '%s' to '%s'\n", path, value);
  }
}

String formatTimestamp(time_t timestamp) {
  struct tm* timeInfo = localtime(&timestamp);
  char buffer[25];
  snprintf(buffer, sizeof(buffer), "%02d-%02d-%04d %02d:%02d:%02d",
           timeInfo->tm_mday, timeInfo->tm_mon + 1, timeInfo->tm_year + 1900,
           timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
  return String(buffer);
}

void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(500);
    esp_task_wdt_reset();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nConnected with IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect.");
  }
}

void syncTime() {
  Serial.print("Configuring time and waiting for sync");
  configTzTime("IST-5:30", "pool.ntp.org");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
    esp_task_wdt_reset();
  }
  Serial.println("\nTime synchronized.");
}


void setupOTA() {
  ArduinoOTA.setHostname("auto-tank");
  ArduinoOTA.onStart([]() {
              Serial.println("OTA Update Start");
            })
    .onEnd([]() {
      Serial.println("\nOTA Update Finished");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA Error[%u]", error);
    });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}
