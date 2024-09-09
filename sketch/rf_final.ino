#include <heltec_unofficial_sh110x.h>
#include <DFRobot_MAX17043.h>
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "String.h"

#define CHARGING_PIN 48
#define BUTTON_PIN 46
#define VIN 47
#define DSET_PIN 7
#define DOUT_PIN 6

// Define frequency range in MHz to scan
#define FREQ_BEGIN 850
#define FREQ_END 950

// Measurement bandwidth
#define BANDWIDTH 467.0

// Define major and minor tickmarks at x MHz
#define MAJOR_TICKS 10
// #define MINOR_TICKS 5

// Use 'PRG' button as the power button, long press to turn off
#define HELTEC_POWER_BUTTON
#include <heltec_unofficial_sh110x.h>

// Include SX1262 patch for spectral scan
#include "C:\\Users\\paula\\Documents\\Arduino\\libraries\\RadioLib\\src\\modules\\SX126x\\patches\\SX126x_patch_scan.h"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t txValue = 0;
long lastMsg = 0;
char rxload[20];

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define STEPS 128
#define SAMPLES 1024
#define MAJOR_TICK_LENGTH 3
#define MINOR_TICK_LENGTH 1
#define X_AXIS_WEIGHT 2
#define HEIGHT RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE  // Adding extra height
#define SCALE_TEXT_TOP (HEIGHT + X_AXIS_WEIGHT + MAJOR_TICK_LENGTH) // Push down scale text
#define STATUS_TEXT_TOP (64 - 8)  // Adjust status text position if needed, closer to the bottom
#define RANGE (float)(FREQ_END - FREQ_BEGIN)
#define SINGLE_STEP (float)(RANGE / STEPS)

uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

DFRobot_MAX17043 battery;

bool power = false;

int buttonPressTime = 0;  // To track how long the button has been pressed
int requiredPressDuration = 50;

bool inAnalysisMode = false;
bool inMonitorMode = false;

bool radioState = false;

int animationStep = 0;
const int animationDelay = 300;

float voltage = 0.0;
int soc = 0;
bool isCharging = false;
String chargingStatus = "";

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
      // Start advertising again to allow auto-reconnection
      pServer->getAdvertising()->start();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      for (int i = 0; i < 20; i++) {
        rxload[i] = 0;
      }
      for (int i = 0; i < rxValue.length(); i++) {
        rxload[i] = (char)rxValue[i];
      }
    }

  }

};

void setupBLE(String BLEName) {
  const char *ble_name = BLEName.c_str();
  BLEDevice::init(ble_name);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void setup() {
  setupBLE("HarvestRF");
  heltec_setup();
  pinMode(CHARGING_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VIN, OUTPUT);
  digitalWrite(VIN, HIGH);

  heltec_display_power(power);

  radio_state(radioState);

  pinMode(DSET_PIN, OUTPUT);
  digitalWrite(DSET_PIN, LOW);
  pinMode(DOUT_PIN, INPUT);

  analogReadResolution(12);

}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);
  isCharging = digitalRead(CHARGING_PIN);
  if (deviceConnected) {
    voltage = battery.readVoltage() / 1000.00;
    soc = estimateSoC(voltage);  
    chargingStatus = isCharging ? "Charging" : "Not Charging";

    char tempVoltageSOCValue[60];
    sprintf(tempVoltageSOCValue, "%.2fV, %d%%, %s", voltage, (int)soc, chargingStatus);
    pCharacteristic->setValue(tempVoltageSOCValue);
    pCharacteristic->notify();

    long now = millis();
    if (now - lastMsg > 100) {
      if (strlen(rxload) > 0) {
        if (strncmp(rxload, "oled_on", 7) == 0) {
          if (!power) {
            power = true;
          }
        }
        if (strncmp(rxload, "analysis", 10) == 0) {
          if (!inAnalysisMode){
            enterAnalysis();
          }
        }
        if (strncmp(rxload, "monitoring", 10) == 0) {
          if (!inMonitorMode) {
            enterMonitor();
          }
        }
        if (strncmp(rxload, "exit", 4) == 0) {
          if (inAnalysisMode || inMonitorMode){
            exitModes();
          }
        }
        if (strncmp(rxload, "predict", 7) == 0) {
          if (inAnalysisMode || inMonitorMode){
            exitModes();
          }
        }
        Serial.println(rxload);
        memset(rxload,0,sizeof(rxload));
      }
      lastMsg = now;
    }
  }

  if (buttonState == LOW) {
    buttonPressTime++;
    if (buttonPressTime > requiredPressDuration) {
      if (!inAnalysisMode && !inMonitorMode) {
        buttonPressTime = 0;
        requiredPressDuration = 1;
        enterAnalysis();
      } else if (inAnalysisMode){
        buttonPressTime = 0;
        requiredPressDuration = 50;
        enterMonitor();
      } else if (inMonitorMode) {
        buttonPressTime = 0;
        requiredPressDuration = 50;
        exitModes();
      }
    }
  } else if (buttonState == HIGH && buttonPressTime > 0) {
    power = true;
    buttonPressTime = 0;
  }

  if (power && !inAnalysisMode && !inMonitorMode) {
    updateDisplay();
    delay(3000);
    power = false;
  } else if (inAnalysisMode) {
    performAnalysis();
  } else if (inMonitorMode) {
    performMonitoring();
  } else {
    power = false;
    display.clearDisplay();
    display.display();
  }
}

void updateDisplay() {
  display.clearDisplay();
  voltage = battery.readVoltage() / 1000.00;
  soc = estimateSoC(voltage);  // Calculate state of charge
  chargingStatus = isCharging ? "Charging" : "";

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Display the text "RF Energy Harvester" at the middle of the screen
  display.setCursor((SCREEN_WIDTH - 6 * 18) / 2 - 3, SCREEN_HEIGHT / 2);
  display.print("RF Energy Harvester");

  // Display Voltage on the first line
  display.setCursor(0, 0);
  display.print(voltage, 2);  // Show voltage with two decimal places
  display.print(" V");

  // Display percentage to the left of the battery icon
  display.setCursor(80 - String(soc).length() + 1, 0);
  display.print(soc);
  display.print("% ");

  display.setCursor(SCREEN_WIDTH  / 2 - 24, (SCREEN_HEIGHT / 2) + 24);
  display.print(chargingStatus);

  // Draw and fill a smaller battery icon
  display.drawRect(103, 0, 20, 10, SH110X_WHITE);
  display.drawRect(123, 2, 2, 6, SH110X_WHITE);
  int fillLevel = map(soc, 0, 100, 0, 18);
  display.fillRect(104, 1, fillLevel, 8, SH110X_WHITE);

  display.display();
}

int estimateSoC(float voltage) {
  const int numPoints = 12;
  const float voltagePoints[numPoints] = {3.65, 3.4, 3.35, 3.33, 3.3, 3.28, 3.25, 3.23, 3.2, 3.13, 3.0, 2.5};
  const int socPoints[numPoints] = {100, 100, 99, 90, 70, 40, 30, 20, 17, 14, 9, 0};

  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage >= voltagePoints[i + 1] && voltage <= voltagePoints[i]) {
      float soc = socPoints[i] + (socPoints[i + 1] - socPoints[i]) 
                  * (voltage - voltagePoints[i]) / (voltagePoints[i + 1] - voltagePoints[i]);
      return (int)round(soc);
    }
  }

  if (voltage >= voltagePoints[0]) {
    return socPoints[0];  // Return 100% if above the highest known voltage
  }
  if (voltage <= voltagePoints[numPoints - 1]) {
    return socPoints[numPoints - 1];  // Return 0% if below the lowest known voltage
  }
  return -1;  // Return -1 in case of an unexpected voltage value
}

void enterAnalysis() {
  digitalWrite(VIN, LOW);
  radioState = true;
  inAnalysisMode = true;
  inMonitorMode = false;
  digitalWrite(DSET_PIN, HIGH);
  power = true;  // Enable power to handle display update
  display.clearDisplay();
  display.setCursor(SCREEN_WIDTH / 2 - 24, SCREEN_HEIGHT / 2 - 10);
  display.print("Entering");
  display.setCursor(SCREEN_WIDTH / 2 - 51, SCREEN_HEIGHT / 2);
  display.print("Spectrum Analysis");
  display.setCursor(SCREEN_WIDTH / 2 - 12, SCREEN_HEIGHT / 2 + 10);
  display.print("Mode");
  display.display();
  delay(3000);
  both.println("Init radio");
  RADIOLIB_OR_HALT(radio.beginFSK(FREQ_BEGIN));
  both.println("Upload patch");
  RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
  both.println("Setting up radio");
  RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setDataShaping(RADIOLIB_SHAPING_NONE));
  both.println("Starting scan");
  display.clearDisplay();  // Clear the SH1106 display
  displayDecorate();       // Add static display elements
}

void enterMonitor() {
  radioState = false;
  inAnalysisMode = false;
  inMonitorMode = true;
  digitalWrite(DSET_PIN, LOW);
  power = true;  // Enable power to handle display update
  display.clearDisplay();
  display.setCursor(SCREEN_WIDTH / 2 - 24, SCREEN_HEIGHT / 2 - 10);
  display.print("Entering");
  display.setCursor(SCREEN_WIDTH / 2 - 45, SCREEN_HEIGHT / 2);
  display.print("Monitoring Mode");
  display.display();
  delay(3000);
  both.println("Init monitor");
  // Additional initialization code for monitoring mode if needed
  display.clearDisplay();  // Clear the SH1106 display
}

void exitModes() {
  digitalWrite(VIN, HIGH);
  radioState = false;
  inAnalysisMode = false;
  inMonitorMode = false;
  digitalWrite(DSET_PIN, LOW);
  digitalWrite(VIN, HIGH);
  display.clearDisplay();
  display.setCursor(SCREEN_WIDTH / 2 - 39, SCREEN_HEIGHT / 2 - 10);
  display.print("Exiting Modes");
  display.display();
  delay(3000);
  updateDisplay();
}
 
void performAnalysis() {
  display.fillRect(0, 0, STEPS, HEIGHT - 5, SH110X_BLACK); 
  for (int x = 0; x < STEPS; x++) {
    float freq = FREQ_BEGIN + (RANGE * ((float)x / STEPS));
    radio.setFrequency(freq);
    radio.spectralScanStart(SAMPLES, 1);
    while (radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE) {
      heltec_delay(1);
    }
    radio.spectralScanGetResult(result);
    for (int y = 0; y < HEIGHT - 5; y++) {
      if (result[y]) {
        display.drawPixel(x, y, SH110X_WHITE);
      }
    }
    heltec_delay(9);  // Prevent SX1262 hangs by delaying between scans
  }
  display.display();  // Update the display with new data
}

void performMonitoring() {
  digitalWrite(VIN, LOW);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Monitoring...");
  display.display();

} 

void displayDecorate() {
    display.setTextSize(1);  // Set text size
    display.setTextColor(SH110X_WHITE);  // Ensure text color is set correctly

    // Calculate text positions dynamically
    String freqBeginStr = " " + String(FREQ_BEGIN);
    String freqEndStr = String(FREQ_END) + " ";
    String scanningStr = "Scanning (in MHz)";

    int freqBeginWidth = 5 * freqBeginStr.length();  // Assume 6 pixels average width per character
    int freqEndWidth = 5 * freqEndStr.length();
    int scanningWidth = 6 * scanningStr.length();

    // Set positions dynamically based on string length
    display.setCursor(0, SCALE_TEXT_TOP);
    display.println(freqBeginStr);

    display.setCursor(128 - freqEndWidth, SCALE_TEXT_TOP);  // Adjust position to right align
    display.println(freqEndStr);

    display.setCursor((128 - scanningWidth) / 2, STATUS_TEXT_TOP);  // Center align
    display.println(scanningStr);

    // Draw the bottom axis and ticks
    display.fillRect(0, HEIGHT - 5, 128, X_AXIS_WEIGHT, SH110X_WHITE);
    drawTicks(MAJOR_TICKS, MAJOR_TICK_LENGTH);
    display.display();  // Make sure to update the display after all changes
}

void drawTicks(float every, int length) {
  float first_tick = FREQ_BEGIN + (every - fmod(FREQ_BEGIN, every));
  for (float tick_freq = first_tick; tick_freq <= FREQ_END; tick_freq += every) {
    int tick = round((tick_freq - FREQ_BEGIN) / SINGLE_STEP);
    display.drawLine(tick, HEIGHT + X_AXIS_WEIGHT - 4, tick, HEIGHT + X_AXIS_WEIGHT + length - 4, SH110X_WHITE);
  }
}



