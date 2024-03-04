/*
This code is the firmware for an ESP32 operated HX711 Scale. 
It communicates over BLE and is meant to imitate a "Decent Scale" for all its intagrations and App functions 
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Preferences.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <string>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include "ICON_BMP.h"


// Add OLED pin definitions
#define OLED_RESET    -1
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define OLED_SDA      21
#define OLED_SCL      22
// Add HX711 pin definitions
#define DT_PIN 25
#define SCK_PIN 26
#define DOUT_PIN 14  // DOUT connection
#define HX711_GAIN_FACTOR 128
// Autotare/weight stability
#define WEIGHT_STABILITY_THRESHOLD 1.0 // Threshold for weight stability in grams
#define TIMER_START_DELAY 5000 // Delay in milliseconds to start the timer after weight increase
#define WEIGHT_INCREASE_THRESHOLD 0.5 // Threshold for weight increase to start the timer
#define STABILITY_DURATION 5000 // Duration of weight stability before considering it stable in milliseconds
#define TIMER_PAUSE_DELAY 2000 // Duration to wait before pausing the timer in milliseconds
// Add button pin definitions
#define BUTTON_BOOT   0
#define BUTTON1_PIN   18
#define BUTTON2_PIN   19
#define DEBOUNCE_DELAY 50
#define AUTOSTANDBY 900000 // Time to go to sleep in milliseconds (15 minutes)
// BLE Commands Define
#define BLE_STARTTIMER  "030B030000000B" // Start the timer
#define BLE_PAUSETIMER  "030B0000000008" // Pause the timer
#define BLE_RESETTIMER  "030B020000000A" // Reset the timer
#define BLE_TARESCALE   "030F000000000C" // Tare the scale
#define BLE_LCDON       "030A0101000009" // Turn LCD ON
#define BLE_LCDOFF      "030A0000000009" // Turn LCD OFF
#define BLE_POWEROFF    "030A020000000B" // Power OFF


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HX711 scale;
Preferences preferences;
SemaphoreHandle_t dataMutex;
bool smartfunctionOn = true;
uint16_t weight;
uint64_t timer_elapsed;
uint64_t startTime; // Variable to hold the start time in microseconds
// Autotare/weight stability
uint16_t lastStableWeight = 0; // Variable to store the last stable weight
uint64_t lastStableTime = 0; // Timestamp of the last stable weight
bool isStable = false; // Flag to indicate if weight is currently stable
bool isTareComplete = false; // Flag to indicate if autotare has been completed
uint64_t lastWeightIncreaseTime = 0; // Timestamp of the last weight increase


const char* serviceUUID = "0000FFF0-0000-1000-8000-00805F9B34FB";
const char* characteristicUUID = "0000FFF4-0000-1000-8000-00805F9B34FB";
const char* deviceName = "DecentScale";
static bool bleClientConnected = false;
static float calibrationFactor;
volatile bool timer_running = false;
volatile uint64_t powerOnTime = esp_timer_get_time();
volatile uint64_t last_button1_time = 0;
volatile uint64_t last_button2_time = 0;

// Function prototypes
void taskReadSensors(void *parameter);
void taskCommunication(void *parameter);
void scaleDisplay(uint8_t mode, uint16_t weight = 0, uint64_t time = 0);
void calibrateScale(float knownWeight);
void buttonboot_pressed();
void button1_pressed();
void button2_pressed();
void poweroff();
void wakeup();
void timer(uint8_t state);
void lcdpower(bool lcdpower);
void sendWeightoverBLE(uint16_t weight);
int hexStringToInt(const std::string& hexStr);
std::string intToHexString(int value);

// Define task handles
TaskHandle_t TaskHandle_ReadSensors = NULL;
TaskHandle_t TaskHandle_Communication = NULL;

// BLE Service
BLECharacteristic *pCharacteristic;
bool newDataReceived = false;

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        newDataReceived = true;
    }
};


void setup() {
  //Serial.begin(115200);
  preferences.begin("inDecentScale", false); // ("name", read-only OFF)
  Wire.begin(OLED_SDA, OLED_SCL);
  scale.begin(DT_PIN, SCK_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  scaleDisplay(0);
  pinMode(DOUT_PIN, INPUT);
  pinMode(BUTTON_BOOT, INPUT_PULLUP); // Set bootbutton as input with internal pull-up resistor
  pinMode(BUTTON1_PIN, INPUT_PULLUP); // Set buttonPin1 as input with internal pull-up resistor
  pinMode(BUTTON2_PIN, INPUT_PULLUP); // Set buttonPin2 as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(BUTTON_BOOT), buttonboot_pressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), button1_pressed, FALLING); // Attach interrupt for button 1
  //attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), button2_pressed, FALLING); // Attach interrupt for button 2
  
  // Read calibration factor from EEPROM
  calibrationFactor = preferences.getFloat("calibrationFactor");


  // Create tasks
    xTaskCreatePinnedToCore(taskReadSensors, "ReadSensors", 4096, NULL, 1, &TaskHandle_ReadSensors, 0);
    xTaskCreatePinnedToCore(taskCommunication, "Communication", 4096, NULL, 2, &TaskHandle_Communication, 1);
}


void loop() {
  // Check if the Standbytime is reached
  if(powerOnTime - esp_timer_get_time() > AUTOSTANDBY){ // go standby
    poweroff();
  }
  delay(1000); // Update every second
}


void taskReadSensors(void *parameter) {
  scale.set_scale(calibrationFactor);
  scale.set_gain(HX711_GAIN_FACTOR);
  delay(100);
  scale.tare();
for (;;) {
  // Read data from the load cell
  xSemaphoreTake(dataMutex, portMAX_DELAY); // Acquire mutex before accessing shared resources
  weight = scale.get_units(3) / calibrationFactor; // Get weight in grams
  xSemaphoreGive(dataMutex); // Release mutex after access

  // Check if the weight is stable
  if (smartfunctionOn){
  if (!isStable && (esp_timer_get_time() - lastStableTime) >= STABILITY_DURATION * 1000) {
      // Check if weight has remained stable for a certain duration
      if (fabs(weight - lastStableWeight) <= WEIGHT_STABILITY_THRESHOLD) {
          // Weight is stable
          lastStableTime = esp_timer_get_time(); // Record the time when weight became stable
          isStable = true; // Mark weight as stable
      }
  } else if (isStable && (esp_timer_get_time() - lastStableTime) >= TIMER_PAUSE_DELAY * 1000) {
      // Check if weight is still stable after the pause delay
      if (fabs(weight - lastStableWeight) <= WEIGHT_STABILITY_THRESHOLD) {
          // Weight remains stable after the delay
          timer(2); // Pause the timer
          isStable = false; // Mark weight as unstable
      } else {
          // Weight has changed, reset stability check
          isStable = false;
      }
  }
  // Check for weight increase after autotare
  if (isTareComplete && weight - lastStableWeight >= WEIGHT_INCREASE_THRESHOLD) {
    // Weight has increased by the threshold after autotare
    // Start the timer after a delay
    if ((esp_timer_get_time() - lastWeightIncreaseTime) >= TIMER_START_DELAY * 1000) {
        timer(1); // Start the timer
    }}
  }

  // Update the elapsed time from timer start event
  while (timer_running) {
      timer_elapsed = esp_timer_get_time() - startTime; // Calculate elapsed time in microseconds
  }
}}

void taskCommunication(void *parameter) {
  BLEDevice::init(deviceName);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(serviceUUID);
  pCharacteristic = pService->createCharacteristic(
    characteristicUUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
    
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();


for (;;) {
  xSemaphoreTake(dataMutex, portMAX_DELAY); // Acquire mutex before accessing shared resources
  // Display on OLED screen
  scaleDisplay(1, weight, timer_elapsed);
  sendWeightoverBLE(weight);
  xSemaphoreGive(dataMutex); // Release mutex after access

  // Handle BLE events here
  if (newDataReceived) {
    // New data received, process it
    std::string value = pCharacteristic->getValue();
    if (value.empty() || value.length() != 12) {
        // Invalid or empty data received
        newDataReceived = false;
    }
    // Process valid data
    const char *data = value.c_str(); // Convert the received data to a C string
        // Check the command
           if (strncmp(data, BLE_STARTTIMER, 12) == 0) {
        // Start the timer
        timer(1);
    } else if (strncmp(data, BLE_PAUSETIMER, 12) == 0) {
        // Pause the timer
        timer(2);
    } else if (strncmp(data, BLE_RESETTIMER, 12) == 0) {
        // Reset the timer
        timer(0);
    } else if (strncmp(data, BLE_TARESCALE, 12) == 0) {
        // Tare Scale
        scale.tare();
    } else if (strncmp(data, BLE_LCDON, 12) == 0) {
        // Turn LCD ON
        lcdpower(true);
    } else if (strncmp(data, BLE_LCDOFF, 12) == 0) {
        // Turn LCD OFF
        lcdpower(false);
    } else if (strncmp(data, BLE_POWEROFF, 12) == 0) {
        // Power OFF
        poweroff();
    } else {
        // Unknown command
    }

    newDataReceived = false; // Reset flag
  }

}}

void scaleDisplay(uint8_t mode, uint16_t weight, uint64_t time) {
    int CHAR_WIDTH, timerWidth, separatorWidth, weightWidth, totalWidth, startX, startY;
    switch(mode) {
      case 0:
        // Bootscreen
        display.ssd1306_command(SSD1306_DISPLAYON);
        display.clearDisplay();
        display.drawBitmap(0, 0, bootscreen, 128, 64, WHITE);
        break;
      case 1:
        // Assuming font size is 1 (default)
        CHAR_WIDTH = 6; // Width of each character in pixels

        // Then use this value to calculate the width of text
        timerWidth = 4 * CHAR_WIDTH + 1; // Assuming 4 characters for the timer (e.g., "10.0")
        separatorWidth = 1; // Width of the separator character '|'
        weightWidth = 5 * CHAR_WIDTH + 1; // Assuming 5 characters for the weight (e.g., "100.0")
        totalWidth = timerWidth + separatorWidth + weightWidth;

        display.clearDisplay();
        if(bleClientConnected == true){
            display.drawBitmap(0, 0, bluetooth_bmpbluetoothtopright, 128, 64, WHITE);
        }
        
        // Set text size according to the available space
        display.setTextSize(1);


        // Calculate the starting coordinates to center the text 
        startX = (110 - totalWidth) / 2;
        startY = (64 - display.height()) / 2;

        // Set cursor position for timer text and Print the timer value
        display.setCursor(startX, startY);
        display.print(time);
        display.print("s");

        // Set cursor position for separator text and Print the separator
        display.setCursor(startX + timerWidth, startY);
        display.print("|");

        // Set cursor position for weight text and Print the weight value
        display.setCursor(startX + timerWidth + separatorWidth, startY);
        display.print(weight);
        display.print("g");

        // Display the content
        display.display();
        break;
      case 2:
        // Calibration Interface
        display.clearDisplay();
        display.println("Calibrate with 50g weight");
        display.display();
        break;
      case 255:
        // Turn off the display
        display.clearDisplay();
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        break;
      default:
        // Handle invalid mode
        // You can choose to do nothing, print an error message, or take other actions.
        break;
    }
}

void calibrateScale(float knownWeight) {
  scaleDisplay(2);
  delay(5000); // Delay to allow time for placing the weight
  long referenceUnit = scale.read(); // Get reference raw value
  calibrationFactor = knownWeight / referenceUnit;

  scale.set_scale(calibrationFactor); // Set calibration factor
  // Save calibration factor to EEPROM
  preferences.putFloat("calibrationFactor", calibrationFactor);
}

void buttonboot_pressed() {
    // Call the calibrateScale function with the appropriate argument
    calibrateScale(50.0f);
}

void button1_pressed() {
    static uint64_t lastButtonPressTime = 0;
    uint64_t currentMillis = millis();
    if (currentMillis - lastButtonPressTime > DEBOUNCE_DELAY) {
        lastButtonPressTime = currentMillis;
        // Add your button handling code here
        if (!timer_running) {
            timer(1);
        } else {
            timer(2);
        }
    }
}

/* void button2_pressed() {
    static uint64_t lastButton2PressTime = 0;
    uint64_t currentMillis = millis();
    if (currentMillis - lastButton2PressTime > DEBOUNCE_DELAY) {
        lastButton2PressTime = currentMillis;
        // Add your button 2 handling code here
    }
} 
*/

void poweroff() {
  attachInterrupt(digitalPinToInterrupt(DOUT_PIN), wakeup, FALLING);
  timer(2);
  lcdpower(0);
  //scale.power_down;
  esp_deep_sleep_start();
}

void wakeup(){
  detachInterrupt(digitalPinToInterrupt(DOUT_PIN));
  //scale.power_up();
  powerOnTime = esp_timer_get_time();
}

void timer(uint8_t state) {
  switch (state) {
    case 0: // Reset timer
      timer_elapsed = 0;
      timer_running = false;
      break;
    case 1: // Start timer
      timer_elapsed = 0;
      startTime = esp_timer_get_time(); // Record the start time in microseconds
      timer_running = true;
      break;
    case 2: // Pause timer
        timer_running = false; // Pause the timer
      break;
    default:
      // Handle invalid state
      break;
  }
}

void lcdpower(bool lcdpower) {
  // Function to enable LCD display
  if(lcdpower){
    scaleDisplay(0); // TURNON LCD display
  } else {
    scaleDisplay(255); // TURNOFF LCD display
  }
}

void sendWeightoverBLE(uint16_t weight) {
    // Get current time using esp_timer_get_time() (sample values used here)
    uint64_t currentTime = esp_timer_get_time(); // Assuming this function returns time in microseconds
    uint64_t minutes = currentTime / (60 * 1000000); // Convert microseconds to minutes
    uint64_t seconds = (currentTime / 1000000) % 60; // Convert microseconds to seconds
    uint64_t milliseconds = (currentTime / 1000) % 1000; // Convert microseconds to milliseconds

    // Byte 1: Constant identifier (03) "Decent"
    std::string encodedPayload = "03";
    
    // Byte 2: Weight (un-/)stable 
    if(isStable){
      encodedPayload += "CE";
    } else{
      encodedPayload += "CA";
    }

    // Byte 3+4: Grams weight * 10
    short weightHighByte = (weight * 10) >> 8;
    short weightLowByte = (weight * 10) & 0xFF;
    encodedPayload += intToHexString(weightHighByte) + intToHexString(weightLowByte);

    // Byte 5: Minutes on
    encodedPayload += intToHexString(minutes);

    // Byte 6: Seconds on (0-59 in hexadecimal)
    encodedPayload += intToHexString(seconds);

    // Byte 7: Milliseconds on (0-9)
    encodedPayload += intToHexString(milliseconds / 100);

    // Byte 8+9: For future use (00 00)
    encodedPayload += "0000";

    // Byte 10: XOR validation
    // Calculate XOR of the first 6 bytes
    int xorValidation = 0;
    for (int i = 0; i < 6; ++i) {
        std::string byteStr = encodedPayload.substr(i * 2, 2);
        xorValidation ^= hexStringToInt(byteStr);
    }
    encodedPayload += intToHexString(xorValidation);

    // Send the payload over BLE
    pCharacteristic->setValue((uint8_t*)encodedPayload.c_str(), encodedPayload.length());
    pCharacteristic->notify(); // Notify the central device
}

// Function to convert an integer to a hexadecimal string
std::string intToHexString(int value) {
    const char hexChars[] = "0123456789ABCDEF";
    std::string result;
    result += hexChars[(value >> 4) & 0xF];
    result += hexChars[value & 0xF];
    return result;
}

// Function to convert a hexadecimal string to an integer
int hexStringToInt(const std::string& hexStr) {
    int result = 0;
    for (char c : hexStr) {
        result <<= 4;
        if (c >= '0' && c <= '9') {
            result += c - '0';
        } else if (c >= 'A' && c <= 'F') {
            result += c - 'A' + 10;
        } else if (c >= 'a' && c <= 'f') {
            result += c - 'a' + 10;
        }
    }
    return result;
}
