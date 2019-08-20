#include <esp_log.h>
#include <BLEDevice.h>
#include "mqtt_client.h"

// Max Number of devices monitored
const int MAX_DEVICES = 10;

// Max duration of BLE scan (in seconds)
const int BLE_SCAN_DURATION = 10;

class DeviceCollector; // forward declaration
class DeviceParser;  // forward declaration

class BleDeviceScanner {
  public:
    // Scan BLE and return true if flora devices are found
    static int Scan(DeviceCollector** collectors);

    class OutStream {
      public:
        OutStream() : _globalreportend(_globalreport) { _globalreport[0] = '\0'; }
        void setDeviceType(const char* type) { strncpy(_type, type, kTypeLength); }
        const char* getDeviceType() const {return _type; }
        void setDeviceAddress(const char* address) { strncpy(_address, address, kBleAddressLength); }
        const char* getDeviceAddress() const {return _address; }
        void setAttributeName(const char* name) { strncpy(_attributeName, name, kNameLength); }
        void setAttributeValue(int intValue) { snprintf(_attributeValue, kValueLength, "%d", intValue); }
        void setAttributeValue(float floatValue) { snprintf(_attributeValue, kValueLength, "%f", floatValue); }
        void setAttributeValue(const char* strValue) { strncpy(_attributeValue, strValue, kValueLength); }
        const char* getAttributeValue() const { return _attributeValue; }
        void add2GlobalReport(const char* iDeviceAddress)  {
          strcpy(_globalreportend, iDeviceAddress);
          _globalreportend+=kBleAddressLength;
          *(_globalreportend++)=',';
          *(_globalreportend++)=' ';
          *(_globalreportend)='\0';
          }
        const char* getGlobalReport() const { return _globalreport; }

        virtual void report();

      protected:
        static const int kTypeLength = 9;
        char _type[kTypeLength+1];
        static const int kBleAddressLength = 17;
        char _address[kBleAddressLength+1];
        static const int kNameLength = 20;
        char _attributeName[kNameLength+1];
        static const int kValueLength = 63;
        char _attributeValue[kValueLength+1];
        char _globalreport[(kBleAddressLength+2)*15+1];
        char* _globalreportend = _globalreport;
    };

    static bool ProcessDevice(BLEClient& client, BLEAddress& address, int tryCount, DeviceParser* parsers, OutStream& out);
};

// void copyBLEAddress(const esp_bd_addr_t from, esp_bd_addr_t to) {
// 	memcpy(to, from, ESP_BD_ADDR_LEN);
// }

class DeviceParser {
  public:

    static const int kNameLength = 20;

    DeviceParser(
      const char name[kNameLength+1],
      const char *service, const char *characteristic,
      bool (*processCharacteristic)(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) = nullptr,
      bool (*mandatory)()  = nullptr,
      bool (*ignore)() = nullptr):
      _service(service), _characteristic(characteristic), _process(processCharacteristic), _mandatory(mandatory), _ignore(ignore) { strncpy(_name, name, DeviceParser::kNameLength); }

    BLEUUID& getServiceUUID() {
      return _service;
    }
    
    BLEUUID& getCharacteristicUUID() {
      return _characteristic;
    }

    const char* getName() const {
      return _name;
    }

    bool processCharacteristic(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) const;
    
    bool ignore() const {
      if (_ignore == nullptr)
        return false;
      else
        return (*_ignore)();
    }
    
    bool mandatory() const {
      if (_mandatory == nullptr)
        return false;
      else
        return (*_mandatory)();
    }

    static bool ExtractIntFromRawData(const std::string& raw_value, int& int_value);
    static bool ExtractFloatFromRawData(const std::string& raw_value, float& float_value);
    static bool IsMandatory();

  private:
    char      _name[kNameLength+1];
    BLEUUID   _service;
    BLEUUID   _characteristic;
    bool      (*_process)(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out);
    bool      (*_mandatory)();
    bool      (*_ignore)();
};

class LAST_PARSER : public DeviceParser {
  private:
    static const char kName[];
  public:
    LAST_PARSER() : DeviceParser(kName, "", "") {}

    static bool IsLast(DeviceParser& e) { return (strncmp(e.getName(), kName, DeviceParser::kNameLength) == 0); }
};


class LAST_COLLECTOR; // forward declaration

class DeviceCollector {
  public:
    int getCount() const {
      return _deviceCount;
    }

    esp_bd_addr_t* getDeviceAddress(int i) {
      if (i < _deviceCount)
        return &_devices[i];
      else
        abort();
    }

    void addToCollection(BLEAdvertisedDevice& advertisedDevice) {
      BLEAddress deviceAddress = advertisedDevice.getAddress();

      if (_deviceCount < MAX_DEVICES) {
        // copyBLEAddress(*(deviceAddress.getNative()), _devices[_deviceCount++]);
        memcpy(_devices[_deviceCount++], *(deviceAddress.getNative()), ESP_BD_ADDR_LEN);
        ESP_LOGD("addToCollection", "Device %s now registered", deviceAddress.toString().c_str());
      }
      else
        ESP_LOGW("addToCollection", "can't register device, no remaining slot");
    }

    virtual bool isMatching(BLEAdvertisedDevice& advertisedDevice) const = 0;

    static LAST_COLLECTOR _LAST;
  private:
    esp_bd_addr_t _devices[MAX_DEVICES];
    int           _deviceCount = 0;
};

class LAST_COLLECTOR : public DeviceCollector {
  public:
    virtual bool isMatching(BLEAdvertisedDevice& advertisedDevice) const { return false; };
};

