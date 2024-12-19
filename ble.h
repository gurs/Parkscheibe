#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string>

// https://www.uuidgenerator.net/

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"



class Ble_service {
public:
  Ble_service() {}

  void init() {
    BLEDevice::init("Parkscheibe");
    pServer = BLEDevice::createServer();

    pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ);


    pCharacteristic->setValue("00:00");
    pService->start();
    pAdvertising = pServer->getAdvertising();
  }

  void deinit() {
    BLEDevice::deinit();
  }

  void advertise_start() {
    pAdvertising->start();
  }

  void advertise_stop() {
    pAdvertising->stop();
  }

  void ble_data(int hour = 0, int minute = 0) {
    String str = String(hour) + ":" + String(minute);
    pCharacteristic->setValue(str);
  }

private:
  BLECharacteristic *pCharacteristic{ nullptr };
  BLEServer *pServer{ nullptr };
  BLEService *pService{ nullptr };
  BLEAdvertising *pAdvertising{ nullptr };
  int advertising_active = 0;
};