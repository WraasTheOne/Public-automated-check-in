#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
String hashMsg(char *payload, char *key);

char *key = "very_SECRET_esp_2222";
String espName = "ESP32-2222";
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


static BLECharacteristic *pCharacteristic;
static String currentToken = "Invailed";

// Callback class for server (optional, handles connections/disconnections)
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    BLEAddress connectedDevice(param->connect.remote_bda);
    Serial.println("Device connected: " + connectedDevice.toString());
    BLEDevice::startAdvertising();
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }
};

// Callback class for characteristic reads and writes
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String challange = pChar->getValue();
    if (challange.length() > 0) {
      Serial.println(challange.c_str());
      String hashedChallange = hashMsg((char*) challange.c_str(), key);
      if (hashedChallange != "ERROR") {
        pChar->setValue(hashedChallange);
      } else {
        Serial.println("ERROR");
      }
      // Update the characteristic value so that when the app reads again, it gets the updated token
    }
  }
  void onRead(BLECharacteristic *pChar) {
    // The read callback is optional in this scenario, but if you want to handle reads explicitly:
    // This will be called whenever the app reads the characteristic.
    Serial.println("App is reading the characteristic");
  }
};

void setup() {

  Serial.begin(115200);

  Serial.println("Starting BLE...");
  BLEDevice::init(espName);//dette er ble navenet
  Serial.println(espName);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY );
  pCharacteristic->setValue(currentToken);

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // optional
  pAdvertising->setMinPreferred(0x12);  // optional
  BLEDevice::startAdvertising();
  Serial.println("BLE service started. Waiting for connections...");
}

void notyfiyTheHash(String hash) {
}

void loop() {
  delay(2000);
}
