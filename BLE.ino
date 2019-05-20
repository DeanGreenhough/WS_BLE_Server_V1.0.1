/*
void BLE_SERVICE() {


  BLEDevice::init("WSoft");                                     // CREATE BLE DEVICE ***MAX 5 LETTERS***

  BLEServer *pServer = BLEDevice::createServer();               // CREATE BLE SERVER
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);  // CREATE BLE SERVICE

  pCharacteristic = pService->createCharacteristic(             // CREATE BLE CHARACTERISTIC
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());                 // CREATE BLE DESCRIPTOR
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);       //important allows connection notfications
  Serial.println("Starting Service & Advertising");
  pService->start();                                             // START THE SERVICE
  pServer->getAdvertising()->start();                            // START ADVERTISING

}
*/
