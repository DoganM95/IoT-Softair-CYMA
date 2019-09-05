// Libraries
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
// #include <esp_pthread.h>
#include <pthread.h>
// #include <queue.h>
// #include "task.h"
// #include <FreeRTOS.h>
// #include <semphr.h>

// Credentials
#include "Credentials/OtaLogin.h"
#include "Credentials/Wifi.h"

// ----------------------------------------------------------------------------
// SETTINGS
// ----------------------------------------------------------------------------

int otaPort = 80;
int touchDetectionThreshold = 18;
int debounceCount = 10;
int debounceDleayCounter = 0;
int i = 0;

// GPIO
// input only
const int shotSensorPin = 34;  // Read: 1 = Free, 0 = Interrupted
const int triggerPin = 35;     // Read: 1 = Shoot, 0 = Stop
// output only
const int firePin = 33;  // Write 1 = enable relay = shoot, else stop

// setting PWM properties
const int freq = 5000;
const int shotChannel = 0;  // Shot Sensor
const int resolution = 8;   // 8 Bit results in 0-255 as Range for Duty Cycle

// Setting debounce delay for mechanical trigger switch/button
int triggerDebounceDelay = 120;

int shotNumber = 0;
int triggerNumber = 0;

// WebServer
// allows you to set the realm of authentication Default:"Login Required"
const char* www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";

WebServer server(otaPort);
const char* serverIndex =
    "<form method='POST' action='/update' enctype='multipart/form-data'><input "
    "type='file' name='update'><input type='submit' value='Update'></form>";

// ----------------------------------------------------------------------------
//  FUNCTION DECLARATIONS
// ----------------------------------------------------------------------------
void shoot();

// ----------------------------------------------------------------------------
// THREADS
// ----------------------------------------------------------------------------
pthread_t triggerThreadHandle;
pthread_t shotdetectionThreadHandle;
pthread_t shootThreadHandle;
pthread_t webserverThreadHandle;

SemaphoreHandle_t triggerSemaphore;
SemaphoreHandle_t shotdetectionSemaphore;
SemaphoreHandle_t shootSemaphore;
SemaphoreHandle_t webServerSemaphore;

pthread_mutex_t mutex;

// pthread_mutex_lock(&mutex);
// Serial.println("mutex locked");
// pthread_mutex_unlock(&mutex);
// Serial.println("mutex unlocked");

xTaskHandle Task1;
int threadCreationResult;
// xTaskHandle Task2;

// xTaskCreatePinnedToCore(&triggerThreadTask, "triggerThread", 10000, NULL, 1,
//                         NULL, 0);

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  // Serial SETUP
  Serial.begin(115200);

  // pthread_create(&pthread_t, NULL, function_to_be_executed, void*
  // input_argument)

  // GPIO SETUP

  // shotsensor
  ledcSetup(shotChannel, freq, resolution);
  ledcAttachPin(shotSensorPin, shotChannel);
  pinMode(shotSensorPin, INPUT_PULLDOWN);
  pinMode(triggerPin, INPUT_PULLDOWN);
  pinMode(firePin, OUTPUT);

  // Wifi Setup
  Serial.println("Waiting for Wifi.");
  WiFi.begin(WIFI_SSID, WIFI_PASSORD);
  while (WiFi.status() != WL_CONNECTED) {  // Wait for Wifi connection
  }
  Serial.printf("Connected to Wifi: %s\n", WIFI_SSID);

  // Thread configurations
  disableCore0WDT();
  disableCore1WDT();

  Serial.print("shotSensor state: normally");
  Serial.println(digitalRead(shotSensorPin));

  // Thread Creations
  if (pthread_create(&webserverThreadHandle, NULL, webServerThreadFunction, NULL) == 0) {
    Serial.println("Thread WebServer successfully started");
  }
  if (pthread_create(&shootThreadHandle, NULL, shootThreadFunction, NULL) == 0) {
    Serial.println("Thread shootFunctionality successfully started");
  }
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {}

// ----------------------------------------------------------------------------
// Actions
// ----------------------------------------------------------------------------
void shoot(boolean state, String automaticMode = "") {  // automaticMode is an optional parameter
  if (state) {
    delay(triggerDebounceDelay / 2);  // debouncing here enables a simple if-else-conditoin before shooting
    if (automaticMode == "semi") {    // registers one shot using speed sensor, delays to complete shot and stops
      while (digitalRead(shotSensorPin) == HIGH) {
        digitalWrite(firePin, HIGH);
      }
      delay(10);
      digitalWrite(firePin, LOW);
      Serial.printf("Shot once - %d\n", ++shotNumber);
    } else if (automaticMode == "full") {  // shoots until either trigger is released or finger stopped touching trigger
      while (digitalRead(triggerPin) == HIGH && touchRead(4) <= touchDetectionThreshold) {
      continueShooting:
        digitalWrite(firePin, HIGH);
      }
      delay(triggerDebounceDelay / 4);
      if (digitalRead(triggerPin) == LOW || touchRead(4) >= touchDetectionThreshold) {
        digitalWrite(firePin, LOW);
      } else {
        goto continueShooting;
      }
    }
  }
}

// ----------------------------------------------------------------------------
// Functions running as threads
// ----------------------------------------------------------------------------
void* webServerThreadFunction(void* param) {
  // Webserver for OTA Updates
  MDNS.begin(WIFI_HOSTNAME);
  ArduinoOTA.begin();

  server.on("/", []() {
    if (!server.authenticate(www_username, www_password)) {
      // Basic Auth Method with Custom realm and Failure Response
      // return server.requestAuthentication(BASIC_AUTH, www_realm,
      // authFailResponse); Digest Auth Method with realm="Login Required" and
      // empty Failure Response return server.requestAuthentication(DIGEST_AUTH);
      // Digest Auth Method with Custom realm and empty Failure Response
      // return server.requestAuthentication(DIGEST_AUTH, www_realm);
      // Digest Auth Method with Custom realm and Failure Response
      return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
    }
    server.sendHeader("Connection, lel", "close");
    server.send(200, "text/html", serverIndex);

    server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    server.on("/update", HTTP_POST,
              []() {
                server.sendHeader("Connection", "close");
                server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
                ESP.restart();
              },
              []() {
                HTTPUpload& upload = server.upload();
                if (upload.status == UPLOAD_FILE_START) {
                  Serial.setDebugOutput(true);
                  Serial.printf("Update: %s\n", upload.filename.c_str());
                  if (!Update.begin()) {  // start with max available size
                    Update.printError(Serial);
                  }
                } else if (upload.status == UPLOAD_FILE_WRITE) {
                  if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                  }
                } else if (upload.status == UPLOAD_FILE_END) {
                  if (Update.end(true)) {  // true to set the size to the current progress
                    Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
                  } else {
                    Update.printError(Serial);
                  }
                  Serial.setDebugOutput(false);
                } else {
                  Serial.printf(
                      "Update Failed Unexpectedly (likely broken connection): "
                      "status=%d\n",
                      upload.status);
                }
              });
  });
  server.begin();
  MDNS.addService("http", "tcp", otaPort);
  Serial.printf("Open http://%s:%i\n", WiFi.localIP(), otaPort);

  while (true) {
    server.handleClient();
    ArduinoOTA.handle();
  }
}

void* shootThreadFunction(void* param) {
  while (true) {
    if (touchRead(4) <= touchDetectionThreshold) {  // Trigger touched by skin
      if (digitalRead(triggerPin) == HIGH) {
        Serial.printf("touched and triggered - %d\n", i++);
        shoot(true, "semi");
      }
    }
  }
}

// Todo:

// 3D Print barrel cover

// implement deepsleep mode (sleep when no shot is detected for 1 minute) and
// connect led's to gpios, so glock lights up a red status led to inidcate
// deepsleep and green when awake

// when touching the trigger, wake the esp from deepsleep mode

// integrate led's to look like a tritium sight (with configurable brightness?
// using photoresistor?)