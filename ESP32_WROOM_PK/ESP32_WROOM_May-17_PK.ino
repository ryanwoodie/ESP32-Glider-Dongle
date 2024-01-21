 // wifi password, BT password, serial speeds
//green gnd, org,brn,ble
#include <WiFi.h>
#include <WiFiUdp.h>
//#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include <esp_gap_ble_api.h>
//#include <esp_bt_defs.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include "esp_system.h"

#define GPS_RX_PIN 18 // GPIO pin number for GPS RX
#define GPS_TX_PIN 19 // GPIO pin number for GPS TX
#define VARIO_RX_PIN 16 // GPIO pin number for Vario RX
#define VARIO_TX_PIN 17 // GPIO pin number for Vario TX
//#define SERIAL_BUFFER_SIZE 512
#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"
//#define UNKNOWN_SERVICE_UUID "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
//#define UNKNOWN_CHARACTISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"
#define DEVICE_INFORMATION_SERVICE_UUID "0000180A-0000-1000-8000-00805F9B34FB"
#define MANUFACTURER_NAME_UUID "00002A29-0000-1000-8000-00805F9B34FB"
#define MODEL_NUMBER_UUID "00002A24-0000-1000-8000-00805F9B34FB"
#define SERIAL_NUMBER_UUID "00002A25-0000-1000-8000-00805F9B34FB"
#define FIRMWARE_REVISION_UUID "00002A26-0000-1000-8000-00805F9B34FB"
#define HARDWARE_REVISION_UUID "00002A27-0000-1000-8000-00805F9B34FB"
#define SOFTWARE_REVISION_UUID "00002A28-0000-1000-8000-00805F9B34FB"

Preferences preferences;

BluetoothSerial SerialBT;
HardwareSerial *gpsSerial;
HardwareSerial *varioSerial;
WiFiUDP udp;

IPAddress localIP(192, 168, 5, 2);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
const int udpPort = 10110;
unsigned long lastVarioDataTime = 0;
unsigned long varioDataTimeout = 10000; // 10 seconds
bool dataFromVarioAvailable = false;
String dataToSend = "";
String dataFromExternal = "";
String ble_external = "";
bool isSendingData = false;

unsigned long lastClientConnectedTime = 0;
const unsigned long maxClientDisconnectedTime = 30; // 30 seconds

bool deviceConnected = false;

struct DeviceSettings {
  bool wifi;
  bool ble;
  bool bt;
  String name;
  int serialbaud;
  int gpsbaud;
  int variobaud;
  bool gps_tx;
  bool vario_uart;
  String wifipassword;
  int btpassword;
};

DeviceSettings settings() {
  DeviceSettings deviceSettings;
  
  delay(500);
  preferences.begin("dongle", false);
  
  String mac = WiFi.macAddress();
  mac.replace(":", ""); // Remove colons from the MAC address
  String fallbackName = "ESP32_" + mac;

  deviceSettings.wifi = preferences.getBool("wifi", true);
  deviceSettings.ble = preferences.getBool("ble", false);
  deviceSettings.bt = preferences.getBool("bt", true);
  deviceSettings.name = preferences.getString("name", fallbackName.c_str());
  deviceSettings.serialbaud = preferences.getInt("serialbaud", 9600);
  deviceSettings.gpsbaud = preferences.getInt("gpsbaud", 9600);
  deviceSettings.variobaud = preferences.getInt("variobaud", 9600);
  deviceSettings.gps_tx = preferences.getBool("gps-tx", true);
  deviceSettings.vario_uart = preferences.getBool("vario-uart", true);
  deviceSettings.wifipassword = preferences.getString("wifipassword", "");
  deviceSettings.btpassword = preferences.getInt("btpassword", 1234);

  preferences.end();

  return deviceSettings;
}

class MyCharacteristicCallback : public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++) {
            char currentChar = (char)rxValue[i];
            ble_external += currentChar;

            if (currentChar == '\n') {
              ble_external.trim();
              sendExternalData(ble_external);
                //Serial.println(ble_external);
              ble_external = "";
            }
        }
    }
}
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Device connected");
       deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->startAdvertising();
    }
};
MyCharacteristicCallback myCharacteristicCallback;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLECharacteristic *pCharacteristicU = NULL;
BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));

DeviceSettings deviceSettings;

void setup() {
  deviceSettings = settings();
  pinMode(VARIO_RX_PIN, INPUT_PULLUP);
  pinMode(VARIO_TX_PIN, INPUT_PULLUP);
  pinMode(GPS_RX_PIN, INPUT_PULLUP);
  pinMode(GPS_TX_PIN, INPUT_PULLUP);
  Serial.begin(115200);
   if (!deviceSettings.gps_tx) {
      gpsSerial = new HardwareSerial(1);
    gpsSerial->begin(deviceSettings.gpsbaud, SERIAL_8N1, GPS_RX_PIN,-1);
     pinMode(GPS_TX_PIN, INPUT);
     }
     else{
    gpsSerial = new HardwareSerial(1);
    gpsSerial->begin(deviceSettings.gpsbaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
     }
  

  if (deviceSettings.vario_uart) {
    varioSerial = new HardwareSerial(2);
    varioSerial->begin(deviceSettings.variobaud, SERIAL_8N1, VARIO_RX_PIN, VARIO_TX_PIN);
  }
  
  if (deviceSettings.wifi) {
   // WiFi.mode(WIFI_AP);
    WiFi.softAP(deviceSettings.name.c_str(), deviceSettings.wifipassword.c_str(), 1, 0, 4);
    delay(500);
    WiFi.softAPConfig(localIP, gateway, subnet);
    delay(500);
    Serial.println("Waiting for a connection...");
    udp.begin(udpPort);
    Serial.print("UDP server running at ");
    Serial.print(localIP);
    Serial.print(":");
    Serial.println(udpPort);

  }
  if (deviceSettings.bt) {
    String btname = deviceSettings.name + "_SPP";
    SerialBT.begin(btname.c_str());
    Serial.println("Bluetooth Classic is active, device name: " + btname);
  }

  if (deviceSettings.ble) {
     setup_BLE(deviceSettings.name);
     Serial.println("Bluetooth LE is active, device name: " + deviceSettings.name);
  }
}

void loop() {

  String dataToSend = getData();
  sendData(dataToSend);
  delay(10);
  String dataFromExternal = getExternalData();
  sendExternalData(dataFromExternal);

}

void setup_BLE(String deviceName) {
  BLEDevice::init(deviceName.c_str());
 //BLEDevice::setMTU(85);  // Set the preferred MTU size to 256
  // set the preferred connection interval range to 50-100 milliseconds

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


  BLEService *pService = pServer->createService(SERVICE_UUID);

//BLEService *pUnknownService = pServer->createService(UNKNOWN_SERVICE_UUID);

  // Create the Device Information Service
  BLEService *pDeviceInformationService = pServer->createService(DEVICE_INFORMATION_SERVICE_UUID);

  // Create a BLE Characteristic for the Manufacturer Name
  BLECharacteristic *pManufacturerNameCharacteristic = pDeviceInformationService->createCharacteristic(
                                                        MANUFACTURER_NAME_UUID,
                                                        BLECharacteristic::PROPERTY_READ
                                                      );
  pManufacturerNameCharacteristic->setValue("HMsoft");

  // Add the remaining characteristics for the Device Information Service
  BLECharacteristic *pModelNumberCharacteristic = pDeviceInformationService->createCharacteristic(
                                                   MODEL_NUMBER_UUID,
                                                   BLECharacteristic::PROPERTY_READ
                                                 );
  pModelNumberCharacteristic->setValue("HM10");

  BLECharacteristic *pSerialNumberCharacteristic = pDeviceInformationService->createCharacteristic(
                                                    SERIAL_NUMBER_UUID,
                                                    BLECharacteristic::PROPERTY_READ
                                                  );
  pSerialNumberCharacteristic->setValue("RW_1");

  BLECharacteristic *pFirmwareRevisionCharacteristic = pDeviceInformationService->createCharacteristic(
                                                        FIRMWARE_REVISION_UUID,
                                                        BLECharacteristic::PROPERTY_READ
                                                      );
  pFirmwareRevisionCharacteristic->setValue("1.0");

  BLECharacteristic *pHardwareRevisionCharacteristic = pDeviceInformationService->createCharacteristic(
                                                        HARDWARE_REVISION_UUID,
                                                        BLECharacteristic::PROPERTY_READ
                                                      );
  pHardwareRevisionCharacteristic->setValue("1.0");

  BLECharacteristic *pSoftwareRevisionCharacteristic = pDeviceInformationService->createCharacteristic(
                                                        SOFTWARE_REVISION_UUID,
                                                        BLECharacteristic::PROPERTY_READ
                                                      );
  pSoftwareRevisionCharacteristic->setValue("1.0");

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );

  UserDescriptor.setValue("HMSoft");
  pCharacteristic->addDescriptor(&UserDescriptor);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharacteristicCallback());
 
  pService->start();
  pDeviceInformationService->start(); // Start the Device Information Service
  //pUnknownService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  //pAdvertising->setMinPreferred(0x50);
  //pAdvertising->setMaxPreferred(0xA0);
  //pAdvertising->setMinPreferred(0x0320);
  pServer->startAdvertising();
}


String getData() {
   String dataToSend = "";
  // Read data from GPS (UART1) and send it to Vario (UART2)
  if (gpsSerial->available()) {
    String dataFromGPS = gpsSerial->readStringUntil('\n');
    dataFromGPS.trim();
    varioSerial->println(dataFromGPS); // Send data to Vario
    if (!dataFromVarioAvailable) {
      dataToSend = dataFromGPS;
    }
  }

  // Read data from Vario (UART2)
  if (varioSerial->available()) {
    String dataFromVario = varioSerial->readStringUntil('\n');
    dataFromVario.trim();
    lastVarioDataTime = millis(); // Update the time when data was last received
    dataToSend = dataFromVario;
    dataFromVarioAvailable = true;
  }
  
    if(deviceSettings.name == "test" && dataToSend.isEmpty()) {
      dataToSend = "$GPRMC,145744.00,A,5217.13499,N,10639.44507,W,0.367,,180423,,,A*63\n";
      delay(1000);
    }
return dataToSend;
  }

 void sendData(String dataToSend) {
  if (millis() - lastVarioDataTime >= varioDataTimeout) {
    dataFromVarioAvailable = false;
  }

  if (!dataToSend.isEmpty()) {
    // Send data to Android phone via USB
    Serial.println(dataToSend);
    SerialBT.println(dataToSend);
    gpsSerial->println(dataToSend);

  if (deviceSettings.wifi)  {
    // Send data via UDP
    udp.beginPacket(IPAddress(192, 168, 5, 3), udpPort);
    udp.println(dataToSend);
    udp.endPacket();
  }
  
    // Send data over BLE
    if (!isSendingData && deviceConnected) {
        isSendingData = true;
      dataToSend += '\n';
      size_t sentData = 0;
      size_t dataSize = dataToSend.length();
      while (sentData < dataSize) {
        size_t chunkSize = min(dataSize - sentData, (size_t)20);
        String chunk = dataToSend.substring(sentData, sentData + chunkSize);
        pCharacteristic->setValue(chunk.c_str());
        pCharacteristic->notify();
        sentData += chunkSize;
        delay(10); // Add a small delay between chunks to give the receiver time to process
        isSendingData = false;
      }
    } 

    // Send data to BluetoothSerial
    dataToSend = ""; // Reset dataToSend after sending
  }

 }

String getExternalData(){
String dataFromExternal = "";

  // Read data from Android phone via USB
  if (Serial.available()) {
    dataFromExternal = Serial.readStringUntil('\n');
    dataFromExternal.trim();
  }

  // Read data from Bluetooth
  if (SerialBT.available()) {
    dataFromExternal = SerialBT.readStringUntil('\n');
    dataFromExternal.trim();
  }

  // Read data from UDP
    if (deviceSettings.wifi)  {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the incoming data
    char incomingData[85];
    int len = udp.read(incomingData, 85);
    if (len > 0) {
      incomingData[len] = '\0';
    }
    dataFromExternal = String(incomingData);
  }
  }
  
 /* if (!ble_external.isEmpty()) {
    dataFromExternal = ble_external;
    ble_external = "";
  }*/
  return dataFromExternal;
}

void sendExternalData(String dataFromExternal){
   if (dataFromExternal != "") {
    if (dataFromExternal == "setup" || dataFromExternal == "SETUP") {
      enterSettingsLoop();
    } else {
  // Send data to GPS (UART1) and Vario (UART2), and Serial if data is available
  
    gpsSerial->println(dataFromExternal); // Send data to GPS
    varioSerial->println(dataFromExternal); // Send data to Vario
    Serial.println(dataFromExternal); // Send data to Serial (Debugging)
    String dataToSend = updateSettings(dataFromExternal);
}
   }
}

String sanitizeName(String input, int maxLength) {
  String sanitized = "";
  for (int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (isalnum(c) || c == '_') {
      sanitized += c;
    }
  }

  if (sanitized.length() > maxLength) {
    sanitized = sanitized.substring(0, maxLength);
  }

  return sanitized;
}

bool parseBoolValue(String value) {
  value.trim();
  value.toLowerCase();
  return (value == "true" || value == "1" || value == "on");
}

String updateSettings(String data) {
  String dataToSend = "";
  data.toLowerCase();
  if (data.startsWith("reboot")) {
  esp_restart(); // Restart the ESP32
  }
  if (data.startsWith("setup")) {
    int firstSpace = data.indexOf(' ');
    int secondSpace = data.indexOf(' ', firstSpace + 1);
    String command;
    String value;

    if (secondSpace != -1) {
      command = data.substring(firstSpace + 1, secondSpace);
      value = data.substring(secondSpace + 1);
    } else {
      command = data.substring(firstSpace + 1);
    }

    preferences.begin("dongle", false);

    if (command == "wifi" || command == "ble" || command == "bt" || command == "gps-tx" || command == "vario-uart") {
      bool boolValue = parseBoolValue(value);
      preferences.putBool(command.c_str(), boolValue);
      dataToSend += "Updated " + command + " value: " + String(boolValue) + "\n";
      sendData("wifi update");
    } else if (command == "name" || command == "wifipassword") {
      String sanitizedName = sanitizeName(value, 20);
      preferences.putString(command.c_str(), sanitizedName.c_str());
      dataToSend += "Updated " + command + " value: " + sanitizedName + "\n";
      sendData("name update");
    } else if (command == "serialbaud" || command == "gpsbaud" || command == "variobaud" || command == "btpassword") {
      int intValue = value.toInt();
      preferences.putInt(command.c_str(), intValue);
      dataToSend += "Updated " + command + " value: " + String(intValue) + "\n";
      sendData("baud update");
    }

    preferences.end();
      sendData(dataToSend);
  }
  return dataToSend;
}


String printCurrentSettings(DeviceSettings currentSettings) {
  String settingsString = "";

  settingsString += "wifi: " + String(currentSettings.wifi) + " (true or false)\n";
  settingsString += "ble: " + String(currentSettings.ble) + " (true or false)\n";
  settingsString += "bt: " + String(currentSettings.bt) + " (true or false)\n";
  settingsString += "name: " + currentSettings.name + " (alphanumeric, max 20 characters)\n";
  settingsString += "gps-tx: " + String(currentSettings.gps_tx) + " (true or false)\n";
  settingsString += "vario-uart: " + String(currentSettings.vario_uart) + " (true or false)\n";
  settingsString += "serialbaud: " + String(currentSettings.serialbaud) + " (integer, e.g., 9600, 19200, 38400, 57600, 115200)\n";
  settingsString += "gpsbaud: " + String(currentSettings.gpsbaud) + " (integer, e.g., 9600, 19200, 38400, 57600, 115200)\n";
  settingsString += "variobaud: " + String(currentSettings.variobaud) + " (integer, e.g., 9600, 19200, 38400, 57600, 115200)\n";
  settingsString += "wifipassword: " + currentSettings.wifipassword + " (alphanumeric, max 20 characters)\n";
  settingsString += "btpassword: " + String(currentSettings.btpassword) + " (integer, 4-6 digits)\n";

  return settingsString;
}

void enterSettingsLoop() {
  delay(100);
  String dataFromExternal = "";
  uint32_t settingsLoopStartTime = millis();
  uint32_t settingsLoopTimeout = 30000; // 30 seconds
  DeviceSettings currentSettings = settings();
  delay(100);
  String settingsString = printCurrentSettings(currentSettings);
  sendData(settingsString); // Send the settings and instructions to all available protocols
  String instructions = "Enter 'setting_name value' (eg. 'wifi false') to update a setting, 'exit' to leave settings mode+reboot\n";
  sendData(instructions);

  while (millis() - settingsLoopStartTime < settingsLoopTimeout) {
    delay(100);
    dataFromExternal = getExternalData();

    if (dataFromExternal != "") {
      // Reset the timer whenever new external data is received
      settingsLoopStartTime = millis();

      if (dataFromExternal == "exit") {
        break;
        esp_restart();
      }

      if (!dataFromExternal.startsWith("setup ")) {
        dataFromExternal = "setup " + dataFromExternal;
      }
      
      String dataToSend = updateSettings(dataFromExternal);
      
    }
  }
   esp_restart(); 
}
