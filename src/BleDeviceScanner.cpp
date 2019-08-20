#include <BleDeviceScanner.h>

const char LAST_PARSER::kName[] = "LAST_PARSER";

bool DeviceParser::IsMandatory() {  
  return true;
}

bool DeviceParser::ExtractIntFromRawData(const std::string& raw_value, int& int_value) {
  bool extraction_ok = true;

  int value_lgth = raw_value.length();

  if (value_lgth == 1) {
    int_value = *((uint8_t*) raw_value.data());
    ESP_LOGD("ExtractIntFromRawData", "int8 value extracted: %i", int_value);
  }
  else if (value_lgth == 2) {
    int_value = *((uint16_t*) raw_value.data());
    ESP_LOGD("ExtractIntFromRawData", "int16 value extracted: %i", int_value);
  }
  else {
    extraction_ok = false;
    ESP_LOGD("ExtractIntFromRawData", "No int extracted from %s", raw_value.c_str());
  }
  
  return extraction_ok;
}

bool DeviceParser::ExtractFloatFromRawData(const std::string& raw_value, float& float_value) {
  bool extraction_ok = true;

  if (raw_value.length() == 4) {
    float_value = *((float *) raw_value.data());
    ESP_LOGD("ExtractFloatFromRawData", "float value extracted: %f", float_value);
  }
  else {
    extraction_ok = false;
    ESP_LOGD("ExtractFloatFromRawData", "No float extracted from %s", raw_value.c_str());
  }
  
  return extraction_ok;
}

bool DeviceParser::processCharacteristic(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) const {
  bool process_ok = false;

  ESP_LOGD("BLE::processCharacteristic", "raw_value is %s", raw_value.c_str());

  if (_process == nullptr) {
    int int_value;
    float float_value;
    if(ExtractIntFromRawData(raw_value, int_value))
      out.setAttributeValue(int_value);
    else if(ExtractFloatFromRawData(raw_value, float_value))
      out.setAttributeValue(float_value);
    else
      out.setAttributeValue(raw_value.c_str());
    process_ok = true;
  }
  else {
    out.setAttributeValue("");
    process_ok = (*_process)(characteristic, raw_value, out);
  }

  if (process_ok)
    if(out.getAttributeValue()[0] == '\0')
      ESP_LOGD("BLE::processCharacteristic", "Processsing OK for %s", _name);
    else
      ESP_LOGD("BLE::processCharacteristic", "Processsing OK for %s, extracted value: %s", _name, out.getAttributeValue());
  else
    ESP_LOGD("BLE::processCharacteristic", "Processing FAILED for %s", _name);
  
  return process_ok;
}

LAST_COLLECTOR DeviceCollector::_LAST;

BLERemoteService* getService(BLEClient& bleClient, BLEUUID& uuid) {

  ESP_LOGD("BLE::getService", "Access service: %s", uuid.toString().c_str());
  BLERemoteService* service = nullptr;

  try {
    service = bleClient.getService(uuid);
  }
  catch (...) {
    // something went wrong
  }
  if (service == nullptr)
    ESP_LOGD("BLE::getService", "Failed to find service");
  else
    ESP_LOGD("BLE::getService", "Found service");

  return service;
}

BLERemoteCharacteristic* getCharacteristic(BLERemoteService& service, BLEUUID& uuid) {

  ESP_LOGD("BLE::getCharacteristic", "Access characteristic: %s", uuid.toString().c_str());
  BLERemoteCharacteristic* characteristic = nullptr;

  try {
    characteristic = service.getCharacteristic(uuid);
  }
  catch (...) {
    // something went wrong
  }

  if (characteristic == nullptr)
    ESP_LOGW("BLE::getCharacteristic", "Failed, skipping characteristic");
  else
  {
    ESP_LOGD("BLE::getCharacteristic", "Found characteristic");
  }

  return characteristic;
}

bool BleDeviceScanner::ProcessDevice(BLEClient& client, BLEAddress& address, int tryCount, DeviceParser* parsers, OutStream& out) {

  bool success = false;

  const char* type = out.getDeviceType();
  out.setDeviceAddress(address.toString().c_str());

  if(client.connect(address)) {
    ESP_LOGI("MAIN", "Processing device at %s (try %i)", out.getDeviceAddress(), tryCount);

    // Process each attribute
    // Lazy accessors to services and characteristics
    BLERemoteService* service = nullptr;
    BLERemoteCharacteristic* characteristic = nullptr;
    std::string raw_value;

    bool mandatoryAttributeNotFound = false;

    // for (int i = 0; i < kExtractNb; i++) {
    for (DeviceParser* parser = parsers; !mandatoryAttributeNotFound && !LAST_PARSER::IsLast(*parser); parser++) {

      // DeviceParser& parser = parsers[i];
      const char* attribute_name = parser->getName();
      out.setAttributeName(attribute_name);

      if (!parser->ignore()) {
        ESP_LOGI(type, "Handling attribute %s", attribute_name);
        mandatoryAttributeNotFound = parser->mandatory();

        //get Service if need be
        BLEUUID& serviceUUID = parser->getServiceUUID();
        if ((service == nullptr) or !serviceUUID.equals(service->getUUID())){
          ESP_LOGI(type, "Access service: %s", serviceUUID.toString().c_str());
          service = getService(client, serviceUUID);
        }

        if (service != nullptr) {

          // get characteristic if need be
          BLEUUID& characteristicUUID = parser->getCharacteristicUUID();
          if ((characteristic == nullptr) or !characteristicUUID.equals(characteristic->getUUID())){
            ESP_LOGI(type, "Access characteristic: %s", characteristicUUID.toString().c_str());
            characteristic = getCharacteristic(*service, characteristicUUID);
            if (characteristic != nullptr)
              raw_value = characteristic->readValue();
          }
          
          if (characteristic != nullptr) {
            if (parser->processCharacteristic(*characteristic, raw_value, out)) {
              success = true;
              mandatoryAttributeNotFound = false;
              out.report();
            }
          }
          else
            ESP_LOGI(type, "No characteristic found at %s", characteristicUUID.toString().c_str());
          
        }
        else
          ESP_LOGI(type, "No Service found at %s", serviceUUID.toString().c_str());
        
      }
      else 
        ESP_LOGI(type, "Ignoring attribute %s", attribute_name);

      if(mandatoryAttributeNotFound) {
        ESP_LOGW(type, "Mandatory attribute not found %s", attribute_name);
        success = false;
      }
    }

    // disconnect from device
    client.disconnect();
    ESP_LOGI(type, "Disconnected from %s", address.toString().c_str());
  }
  else {
    ESP_LOGI("MAIN", "Failed to connect to %s (try %i)", out.getDeviceAddress(), tryCount);
    success = false;
  }

  if(success)
    out.add2GlobalReport(address.toString().c_str());

  return success;
}



int BleDeviceScanner::Scan(DeviceCollector** collectors) {
  ESP_LOGI("BleScan", "Scan BLE, looking for Devices");

  // detect and register Flora devices during BLE scan
  class DeviceSorter: public BLEAdvertisedDeviceCallbacks {
    public:
      DeviceSorter(DeviceCollector** collectors) : _collectors(collectors) { }

      void onResult(BLEAdvertisedDevice advertisedDevice)
      {
        for (DeviceCollector** iter = _collectors; *iter != &DeviceCollector::_LAST; iter++) {
          if ((*iter)->isMatching(advertisedDevice)) {
            ESP_LOGI("DeviceSorter", "Device %s is matchting", advertisedDevice.getAddress().toString().c_str());
            (*iter)->addToCollection(advertisedDevice);
          }
        }
      }

    private:
      DeviceCollector** _collectors;
  };

  BLEScan* scan = BLEDevice::getScan();
  DeviceSorter sorter(collectors);
  scan->setAdvertisedDeviceCallbacks(&sorter);

  scan->start(BLE_SCAN_DURATION);

  int deviceCount = 0;
  for (DeviceCollector** iter = collectors; *iter != &DeviceCollector::_LAST; iter++)
    deviceCount += (*iter)->getCount();

  ESP_LOGI("BleScan", "Number of devices detected: %i", deviceCount);
  return (deviceCount);
}


void BleDeviceScanner::OutStream::report() {
  ESP_LOGI("DeviceOutStream::report", "%s/%s/%s : %s", _type, _address, _attributeName, _attributeValue);
}

