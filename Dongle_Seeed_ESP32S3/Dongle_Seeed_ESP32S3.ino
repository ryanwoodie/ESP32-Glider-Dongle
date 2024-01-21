// Seeed ESP32S3 based dongle
// 
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "esp_system.h"

#define GPS_RX_PIN 3 // GPIO pin number for GPS RX white USB wire
#define GPS_TX_PIN 4 // GPIO pin number for GPS TX green USB wire
#define VARIO_RX_PIN 1// GPIO pin number for Vario RX
#define VARIO_TX_PIN 2 // GPIO pin number for Vario TX
#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"
//#define DEVICE_INFORMATION_SERVICE_UUID "0000180A-0000-1000-8000-00805F9B34FB"

Preferences preferences;

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
String BLEclientData = ""; // Global string to store received data
String BLEclientDataNoVario = ""; // Global string to store received data
bool isSendingData = false;
unsigned long reconnectIntervals[] = {1000,5000,10000, 30000, 60000, 120000, 300000};
size_t currentIntervalIndex = 0;
unsigned long lastReconnectAttempt = 0;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
unsigned long lastClientConnectedTime = 0;
const unsigned long maxClientDisconnectedTime = 30; // 30 seconds
bool deviceConnected = false;
unsigned long previousMillis = 0;
unsigned long interval = 1000;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");

struct DeviceSettings {
  bool wifi;
  bool ble;
  bool bt;
  bool ble_client;
  String name;
  int serialbaud;
  int gpsbaud;
  int variobaud;
  bool gps_tx;
  bool vario_uart;
  String bleclientname;
  int btpassword;
};

DeviceSettings settings() {
  DeviceSettings deviceSettings;
  
  delay(500);
  preferences.begin("dongle", false);
  
  String mac = WiFi.macAddress();
  mac.replace(":", ""); // Remove colons from the MAC address
  String fallbackName = "ESP32_" + mac;

  deviceSettings.wifi = preferences.getBool("wifi", false);
  deviceSettings.ble = preferences.getBool("ble", false);
  deviceSettings.bt = preferences.getBool("bt", true);
  deviceSettings.ble_client = preferences.getBool("ble_client", false);
  deviceSettings.name = preferences.getString("name", fallbackName.c_str());
  deviceSettings.serialbaud = preferences.getInt("serialbaud", 115200);
  deviceSettings.gpsbaud = preferences.getInt("gpsbaud", 19200);
  deviceSettings.variobaud = preferences.getInt("variobaud", 9600);
  deviceSettings.gps_tx = preferences.getBool("gps-tx", true);
  deviceSettings.vario_uart = preferences.getBool("vario-uart", true);
  deviceSettings.bleclientname = preferences.getString("bleclientname", "SoftRF1234");
  deviceSettings.btpassword = preferences.getInt("btpassword", 1234);

  preferences.end();

  return deviceSettings;
}
DeviceSettings deviceSettings;

 class MyCharacteristicCallback : public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++) {
            isSendingData = true;
            char currentChar = (char)rxValue[i];
            ble_external += currentChar;

            //if (currentChar == '\n') {
              //ble_external.trim();
              //sendExternalData(ble_external);
                //Serial.println(ble_external);
              //ble_external = "";
            //}
            
        }
        isSendingData = false;
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

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
   Serial.println("notifyBLEclient");
    for (size_t i = 0; i < length; i++) {
    isSendingData = true;
        char currentChar = static_cast<char>(pData[i]);
        BLEclientData += currentChar;
    }
    isSendingData = false;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(85); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    Serial.println(ESP.getFreeHeap());

  // Check if the device has a name and if the name matches the desired device name.
  if (advertisedDevice.getName().c_str() && String(advertisedDevice.getName().c_str()) == deviceSettings.bleclientname.c_str()) {

    // We have found a device with the desired name, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
      
      if (myDevice == nullptr) {
        Serial.println("No suitable device found");
        return;
      }
    
    } // Found our server
  } // Found our device
 }// onResult
}; // MyAdvertisedDeviceCallbacks

MyCharacteristicCallback myCharacteristicCallback;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLECharacteristic *pCharacteristicU = NULL;
BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));


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
  

 // if (deviceSettings.vario_uart) {
    varioSerial = new HardwareSerial(2);
    varioSerial->begin(deviceSettings.variobaud, SERIAL_8N1, VARIO_RX_PIN, VARIO_TX_PIN);
 // }
  
  if (deviceSettings.wifi) {
   // WiFi.mode(WIFI_AP);
    WiFi.softAP(deviceSettings.name.c_str(), "12345678", 1, 0, 4);
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

  if (deviceSettings.ble) {
     setup_BLE(deviceSettings.name);
     Serial.println("Bluetooth LE is active, device name: " + deviceSettings.name);
     delay(200);
  }

if (deviceSettings.ble_client){
  if(!deviceSettings.ble){
    BLEDevice::init(deviceSettings.name.c_str());
  }
  startBLEScan();
}
 
}

void loop() {

  if(deviceSettings.ble_client){
  if(!connected){
    BLEclientConnect();
  }
  }
  String dataToSend = getData();
  //delay(50);
  sendData(dataToSend);
  //delay(50);
  String dataFromExternal = getExternalData();
  //delay(50);
  sendExternalData(dataFromExternal);
  delay(10);



  // Your existing loop code here...
 
if(deviceSettings.name.indexOf("mem") >= 0){
unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Free memory: ");
    Serial.println(ESP.getFreeHeap());
  }
}

}


String getData() {
   String dataToSend = "";
  // Read data from GPS (UART1) and send it to Vario (UART2)
  if (gpsSerial->available()) {
    String dataFromGPS = gpsSerial->readStringUntil('\n');
    dataFromGPS.trim();
    //if(deviceSettings.name == "test123"){
   dataFromGPS = processGPSData(dataFromGPS);
      varioSerial->println(dataFromGPS); // Send data to Vario
    //}
    //Serial.println("gps: " + datafromGPS);
    if (!dataFromVarioAvailable) {
      dataToSend = dataFromGPS;
       Serial.println("gps-test");
      return dataToSend;
    }
  }



if (!BLEclientData.isEmpty() && !isSendingData)
{
  BLEclientData.trim();
  dataToSend = BLEclientData;
  BLEclientData = "";
  if(dataFromVarioAvailable){
    dataToSend = processGPSData(dataToSend);
    varioSerial->println(dataToSend);
    dataToSend = "";
    return dataToSend;
  }
  else {
    return dataToSend;
  }
}


  // Read data from Vario (UART2)
  if (varioSerial->available()) {
    String dataFromVario = varioSerial->readStringUntil('\n') + '\n';
    dataFromVario.trim();
    lastVarioDataTime = millis(); // Update the time when data was last received
    dataToSend = dataFromVario;
    dataFromVarioAvailable = true;
  }

  if(deviceSettings.name.indexOf("testdata") >= 0 && dataToSend.isEmpty()) {
      dataToSend = "$GPRMC,145744.00,A,5217.13499,N,10639.44507,W,0.367,,180423,,,A*63";
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
    //SerialBT.println(dataToSend);
    //gpsSerial->print(dataToSend);

  if (deviceSettings.wifi)  {
    // Send data via UDP
    udp.beginPacket(IPAddress(192, 168, 5, 3), udpPort);
    udp.println(dataToSend);
    udp.endPacket();
  }
  
 if (!isSendingData && deviceConnected) {
    isSendingData = true;
    dataToSend += '\n';
    size_t sentData = 0;
    size_t dataSize = dataToSend.length();
    while (sentData < dataSize && deviceConnected) {
        size_t chunkSize = min(dataSize - sentData, (size_t)20);
        String chunk = dataToSend.substring(sentData, sentData + chunkSize);
        pCharacteristic->setValue(chunk.c_str());
        pCharacteristic->notify();
        sentData += chunkSize;
        delay(5);
    }
    isSendingData = false;
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
    return dataFromExternal;
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
    dataFromExternal.trim();
    return dataFromExternal;
  }
  }
  
  if (!ble_external.isEmpty()) {
    dataFromExternal = ble_external;
    ble_external = "";
    dataFromExternal.trim();
    return dataFromExternal;
  }
  return dataFromExternal;
}

void sendExternalData(String dataFromExternal){
   if (dataFromExternal != "") {
    if (dataFromExternal == "setup" || dataFromExternal == "SETUP") {
      enterSettingsLoop();
    } else {
  // Send data to GPS (UART1) and Vario (UART2), and Serial if data is available
    dataFromExternal.trim();
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
  String dataCased = data;
  data.toLowerCase();
  if (data.startsWith("reboot")) {
  esp_restart(); // Restart the ESP32
  }
  if (data.startsWith("setup")) {
    int firstSpace = data.indexOf(' ');
    int secondSpace = data.indexOf(' ', firstSpace + 1);
    String command;
    String value;
    String valueCased;

    if (secondSpace != -1) {
      command = data.substring(firstSpace + 1, secondSpace);
      value = data.substring(secondSpace + 1);
      valueCased = dataCased.substring(secondSpace + 1);
    } else {
      command = data.substring(firstSpace + 1);
    }

    preferences.begin("dongle", false);

    if (command == "wifi" || command == "ble" || command == "bt" || command == "gps-tx" || command == "vario-uart" || command == "ble_client") {
      bool boolValue = parseBoolValue(value);
      preferences.putBool(command.c_str(), boolValue);
      dataToSend += "Updated " + command + " value: " + String(boolValue) + "\n";
      sendData("settings update");
    } else if (command == "name") {
      String sanitizedName = sanitizeName(value, 20);
      preferences.putString(command.c_str(), sanitizedName.c_str());
      dataToSend += "Updated " + command + " value: " + sanitizedName + "\n";
      sendData("device name update");
    } else if (command == "bleclientname") {
      preferences.putString(command.c_str(), valueCased.c_str());
      dataToSend += "Updated " + command + " value: " + valueCased + "\n";
      sendData("bleclient name update");
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
  settingsString += "ble_client: " + String(currentSettings.ble_client) + " (true or false)\n";
  settingsString += "gps-tx: " + String(currentSettings.gps_tx) + " (true or false)\n";
  settingsString += "vario-uart: " + String(currentSettings.vario_uart) + " (true or false)\n";
  settingsString += "serialbaud: " + String(currentSettings.serialbaud) + " (integer, e.g., 9600, 19200, 38400, 57600, 115200)\n";
  settingsString += "gpsbaud: " + String(currentSettings.gpsbaud) + " (integer, e.g., 9600, 19200, 38400, 57600, 115200)\n";
  settingsString += "variobaud: " + String(currentSettings.variobaud) + " (integer, e.g., 9600, 19200, 38400, 57600, 115200)\n";
  settingsString += "bleclientname: " + currentSettings.bleclientname + " (alphanumeric name of BLE device)\n";
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
  String instructions = "Hello Jay! Enter 'setting_name value' (eg. 'wifi false') to update a setting, 'exit' to leave settings mode+reboot\n";
  sendData(instructions);
  delay(1000);

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

void BLEclientConnect(){
 if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
      currentIntervalIndex = 0; // Reset interval index to the first one after a successful connection
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  } else if (!connected) { // If not connected, check if it's time to reconnect
    unsigned long now = millis();
    if (now - lastReconnectAttempt >= reconnectIntervals[currentIntervalIndex]) {
    Serial.println("Restart scan");     
    startBLEScan();
      lastReconnectAttempt = now;

      // Move to the next interval, unless it's already at the last one (every 5 min)
      if (currentIntervalIndex < sizeof(reconnectIntervals) / sizeof(reconnectIntervals[0]) - 1) {
        currentIntervalIndex++;
      }
    }
  }
}

void setup_BLE(String deviceName) {
  BLEDevice::init(deviceName.c_str());
  //BLEDevice::setMTU(85);  // Set the preferred MTU size to 256
  // set the preferred connection interval range to 50-100 milliseconds

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

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

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  //pAdvertising->setMinPreferred(0x50);
  //pAdvertising->setMaxPreferred(0xA0);
  //pAdvertising->setMinPreferred(0x0320);
  pServer->startAdvertising();
}

String processGPSData(String data) {
    if (data.startsWith("$GN")) {
        data = "$GP" + data.substring(3);
        data = recalculateChecksum(data);
    }
    return data;
}


String recalculateChecksum(String sentence) {
  int checksum = 0;
  for (size_t i = 1; i < sentence.length(); ++i) {
    char currentChar = sentence[i];
    if (currentChar == '*') {
      break;
    }
    checksum ^= currentChar;
  }

  String newChecksum = String(checksum, HEX);
  newChecksum.toUpperCase();
  if (newChecksum.length() < 2) {
    newChecksum = "0" + newChecksum;
  }

  size_t asteriskPos = sentence.indexOf('*');
  return sentence.substring(0, asteriskPos + 1) + newChecksum;
}

void startBLEScan() {
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(300);
  pBLEScan->setWindow(200);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(3, false);
  delay(50);
}