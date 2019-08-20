
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include <BLEDevice.h>

#include "config.h"
#include <BleDeviceScanner.h>
#include <cmath>

// boot count used to check if battery status should be read
static int bootCount = 0;

class FloraDevices : public DeviceCollector {
  public:
    virtual bool isMatching(BLEAdvertisedDevice& advertisedDevice) const {
      return (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(rootServiceDataUUID));
    }

    static const BLEUUID rootServiceDataUUID;
};

// Root service for Flora Devices
const BLEUUID FloraDevices::rootServiceDataUUID((uint16_t) 0xfe95);


class ParrotDevices : public DeviceCollector {
  public:
    virtual bool isMatching(BLEAdvertisedDevice& advertisedDevice) const {
      return (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(kServiceUUID));
    }
  private:
    static const BLEUUID kServiceUUID;
};

const BLEUUID ParrotDevices::kServiceUUID("39e1f900-84a8-11e2-afba-0002a5d5c51b");

class FloraExtractor {
  public:
    static bool setWriteMode(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      // write the magic data
      uint8_t buf[2] = {0xA0, 0x1F};
      bool ok = characteristic.writeValue(buf, 2, true);

      if (ok) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        characteristic.getRemoteService()->removeCharacteristics();
      }

      return ok;
    }

    static bool ignore() {return false; }

    static bool extractTemperature(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      float float_value = float(*((uint16_t*) raw_value.data())) / 10;
      out.setAttributeValue(float_value);

      return (float_value < 200);
    }

    static bool extractMoisture(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      int int_value = raw_value.data()[7];
      out.setAttributeValue(int_value);

      return (int_value < 200);
    }

    static bool extractLight(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      int int_value = *((uint16_t*) &(raw_value.data()[3]));
      out.setAttributeValue(int_value);

      return true;
    }

    static bool extractConductivity(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      int int_value = *((uint16_t*) &(raw_value.data()[8]));
      out.setAttributeValue(int_value);

      return true;
    }

    static bool extractBattery(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      int int_value = raw_value.data()[0];
      out.setAttributeValue(int_value);

      return true;
    }
};

class ParrotExtractor {
  public:

    static bool extractTemperature(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      int raw_int;
      if(DeviceParser::ExtractIntFromRawData(raw_value, raw_int)) {
        ESP_LOGI("ParrotExtractor", "extractTemperature from %d (raw %s)", raw_int, raw_value.c_str());
        float float_value = 0.00000003044 * std::pow(raw_int, 3) - 0.00008038 * std::pow(raw_int, 2) + raw_int * 0.1149 - 30.449999999999999;
        out.setAttributeValue(float_value);
        return true;
      }
      else
      {
        ESP_LOGI("ParrotExtractor", "extractTemperature failed from raw %s", raw_value.c_str());
        return false;
      } 
    }

    static bool extractMoisture(BLERemoteCharacteristic& characteristic, const std::string& raw_value, BleDeviceScanner::OutStream& out) {
      int raw_int;
      if (DeviceParser::ExtractIntFromRawData(raw_value, raw_int)) {
        ESP_LOGI("ParrotExtractor", "extractMoisture from %d (raw %s)", raw_int, raw_value.c_str());
        float float_value = 11.4293 + (0.0000000010698 * std::pow(raw_int, 4) - 0.00000152538 * std::pow(raw_int, 3) +  0.000866976 * std::pow(raw_int, 2) - 0.169422 * raw_int);
        float_value = 100.0 * (0.0000045 * std::pow(float_value, 3.0) - 0.00055 * std::pow(float_value, 2.0) + 0.0292 * float_value - 0.053);
        out.setAttributeValue(float_value);
        return true;
      }
      else
      {
        ESP_LOGI("ParrotExtractor", "extractMoisture failed from raw %s", raw_value.c_str());
        return false;
      }
    }
};




static const char *TAG = "Device Scanner";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      esp_wifi_connect();
      xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}

static void wifi_init(void)
{
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  wifi_config_t wifi_config = {
    .sta = {
      {.ssid = CONFIG_WIFI_SSID},
      {.password = CONFIG_WIFI_PASSWORD}
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]", CONFIG_WIFI_SSID, "******");
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG, "Waiting for wifi");
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
    }
    return ESP_OK;
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
  const esp_mqtt_client_config_t mqtt_cfg = {
    event_handle : &mqtt_event_handler,
    host : MQTT_HOST,
    uri: MQTT_URI,
    port : MQTT_PORT
  };

  ESP_LOGI(TAG, "INIT MQTT");

  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  ESP_ERROR_CHECK(esp_mqtt_client_start(client));

  vTaskDelay(3000 / portTICK_PERIOD_MS);

  return client;
}

class Out2MQTT : public BleDeviceScanner::OutStream {
  public:
    Out2MQTT(esp_mqtt_client_handle_t mqttClient) : _mqttClient(mqttClient) {}
    virtual void report();
  private:
    esp_mqtt_client_handle_t _mqttClient;
};

void Out2MQTT::report() {
  BleDeviceScanner::OutStream::report();
  if(_attributeValue[0] != '\0') {
    char header[kTypeLength+1+kBleAddressLength+1+DeviceParser::kNameLength+1];
    sprintf(header, "%s/%s/%s", _type, _address, _attributeName);

    esp_mqtt_client_publish(_mqttClient, header, _attributeValue, 0, 0, 0);
  }
}

void hibernate() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ll);
  ESP_LOGI(TAG, "Going to sleep now for %ds....", SLEEP_DURATION);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  fflush(stdout);
  esp_deep_sleep_start();
}

void hibernateIfITO(void *parameter) {
  ESP_LOGI(TAG, "I plan to sleep in not more than %ds", ITO_APP);
  vTaskDelay(ITO_APP * 1000 / portTICK_PERIOD_MS);

  ESP_LOGE(TAG, "Something got stuck, entering emergency hibernate...");
  hibernate();
}

void killIfStalled(void *parameter) {
  ESP_LOGI(TAG, "The BLE exploration can't take more than %ds", ITO_SERVICE);
  vTaskDelay(ITO_SERVICE * 1000 / portTICK_PERIOD_MS);

  ESP_LOGE(TAG, "Service got stalled, rebooting...");
  // Report in MQTT the end (and failure) of the transaction
  {
    esp_mqtt_client_handle_t mqttClient = (esp_mqtt_client_handle_t) parameter;
    char espStateMqttHeader[3+1+23+1+5+1];
    char espResultMqttHeader[3+1+23+1+6+1];
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    sprintf(espStateMqttHeader, "ESP/%d:%d:%d:%d:%d:%d/state", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(espResultMqttHeader, "ESP/%d:%d:%d:%d:%d:%d/result", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_mqtt_client_publish(mqttClient, espResultMqttHeader, "ITO", 0, 0, 0);
    esp_mqtt_client_publish(mqttClient, espStateMqttHeader, "OFF", 0, 0, 0);
  }

  for (int i = 5; i >= 0; i--) {
      printf("Restarting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  printf("Restarting now.\n");

  fflush(stdout);
  esp_restart();
}

// static void my_gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param) {
//   ESP_LOGD(TAG, "custom gattc event handler, event: %d", (uint8_t)event);
//   if (event == ESP_GATTC_DISCONNECT_EVT)
//     ESP_LOGI(TAG, "Disconnect reason: %d", (int)param->disconnect.reason);
// }

extern "C" void app_main()
{
  for (int i = 5; i >= 0; i--) {
      printf("Starting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  nvs_flash_init();

  // increase boot count
  bootCount++;

    // create a hibernate task in case something gets stuck
  TaskHandle_t itoTaskHandle = nullptr;
  xTaskCreate(hibernateIfITO, "Application ITO", 4096, nullptr, 1, &itoTaskHandle);


  ESP_LOGI("MAIN", "Initialize BLE client...");
  // BLEDevice::setCustomGattcHandler(my_gattc_event_handler);  // before BLEDevice::init();
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  ParrotDevices parrots;
  FloraDevices floras;
  DeviceCollector* collectors[]  = {
    &floras,
    &parrots,
    &DeviceCollector::_LAST
  };
  
  int deviceFound = BleDeviceScanner::Scan(collectors);
  if(deviceFound > 0) {

    wifi_init();
    esp_mqtt_client_handle_t mqttClient = mqtt_app_start();
    Out2MQTT out(mqttClient);

    char espStateMqttHeader[3+1+23+1+5+1];
    char espResultMqttHeader[3+1+23+1+6+1];
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    ESP_LOGI(TAG, "[APP] mac: %d:%d:%d:%d:%d:%d", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(espStateMqttHeader, "ESP/%d:%d:%d:%d:%d:%d/state", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(espResultMqttHeader, "ESP/%d:%d:%d:%d:%d:%d/result", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_mqtt_client_publish(mqttClient, espStateMqttHeader, "ON", 0, 0, 0);

    TaskHandle_t serviceItoTaskHandle = nullptr;
    xTaskCreate(killIfStalled, "Service ITO", 4096, mqttClient, 1, &serviceItoTaskHandle);

    BLEClient* bleClient = BLEDevice::createClient();

    if (floras.getCount() > 0)
    {
      // process Mi Flora devices

      DeviceParser parsers[] = {
        DeviceParser("set write mode", "00001204-0000-1000-8000-00805f9b34fb", "00001a00-0000-1000-8000-00805f9b34fb", &FloraExtractor::setWriteMode, &DeviceParser::IsMandatory),
        DeviceParser("temperature", "00001204-0000-1000-8000-00805f9b34fb", "00001a01-0000-1000-8000-00805f9b34fb", &FloraExtractor::extractTemperature, &DeviceParser::IsMandatory),
        DeviceParser("moisture", "00001204-0000-1000-8000-00805f9b34fb", "00001a01-0000-1000-8000-00805f9b34fb", &FloraExtractor::extractMoisture),
        DeviceParser("light", "00001204-0000-1000-8000-00805f9b34fb", "00001a01-0000-1000-8000-00805f9b34fb", &FloraExtractor::extractLight),
        DeviceParser("conductivity", "00001204-0000-1000-8000-00805f9b34fb", "00001a01-0000-1000-8000-00805f9b34fb", &FloraExtractor::extractConductivity),
        DeviceParser("battery", "00001204-0000-1000-8000-00805f9b34fb", "00001a02-0000-1000-8000-00805f9b34fb", &FloraExtractor::extractBattery),
        LAST_PARSER()
      };

      out.setDeviceType("MiFlora");

      for (int i = 0; i < floras.getCount(); i++) {
        int tryCount = 0;
        BLEAddress deviceAddress(*floras.getDeviceAddress(i));
    
        while (tryCount < RETRY) {
          tryCount++;
          ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
          if (BleDeviceScanner::ProcessDevice(*bleClient, deviceAddress, tryCount, parsers, out)) {
            break;
          }
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }

    if(parrots.getCount() > 0)
    {
      // process Parrot devices

      DeviceParser parsers[] = {
        DeviceParser("battery", "0000180f-0000-1000-8000-00805f9b34fb", "00002a19-0000-1000-8000-00805f9b34fb"), // battery_4b
        DeviceParser("name", "00001800-0000-1000-8000-00805f9b34fb", "00002a00-0000-1000-8000-00805f9b34fb"),  // name_03
        DeviceParser("soil_temperature", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa03-84a8-11e2-afba-0002a5d5c51b", &ParrotExtractor::extractTemperature), // soil_temperature_34
        DeviceParser("air_temperature", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa04-84a8-11e2-afba-0002a5d5c51b", &ParrotExtractor::extractTemperature),  // air_temperature_37
        DeviceParser("air_temperature_cal", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa0a-84a8-11e2-afba-0002a5d5c51b"), // air_temperature_cal_44
        DeviceParser("moisture", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa05-84a8-11e2-afba-0002a5d5c51b", &ParrotExtractor::extractMoisture), // moisture_3a
        DeviceParser("moisture_cal", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa09-84a8-11e2-afba-0002a5d5c51b"), // moisture_cal_41
        DeviceParser("light", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa01-84a8-11e2-afba-0002a5d5c51b"),  // light_25
        DeviceParser("dli_cal", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa0b-84a8-11e2-afba-0002a5d5c51b"),  // dli_cal_47
        DeviceParser("conductivity", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa02-84a8-11e2-afba-0002a5d5c51b"),  // conductivity_31
        DeviceParser("moisture_rj", "39e1fa00-84a8-11e2-afba-0002a5d5c51b", "39e1fa02-84a8-11e2-afba-0002a5d5c51b", &ParrotExtractor::extractMoisture),  // moisture_rj_31
        DeviceParser("watertank_Level", "39e1f900-84a8-11e2-afba-0002a5d5c51b", "39e1f907-84a8-11e2-afba-0002a5d5c51b"),  // watertank_Level_8b
        DeviceParser("watering_mode", "39e1f900-84a8-11e2-afba-0002a5d5c51b", "39e1f90d-84a8-11e2-afba-0002a5d5c51b"),  // watering_mode_90
        DeviceParser("watering_status", "39e1f900-84a8-11e2-afba-0002a5d5c51b", "39e1f912-84a8-11e2-afba-0002a5d5c51b"),  // watering_status_9a
        LAST_PARSER()
      };

      out.setDeviceType("ParrotPot");

      for (int i = 0; i < parrots.getCount(); i++) {
        int tryCount = 0;
        BLEAddress deviceAddress(*parrots.getDeviceAddress(i));
    
        while (tryCount < RETRY) {
          tryCount++;
          ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
          if (BleDeviceScanner::ProcessDevice(*bleClient, deviceAddress, tryCount, parsers, out)) {
            break;
          }
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }

    vTaskDelete(serviceItoTaskHandle);

    // Report before sleeping
    esp_mqtt_client_publish(mqttClient, espStateMqttHeader, "OFF", 0, 0, 0);
    esp_mqtt_client_publish(mqttClient, espResultMqttHeader, out.getGlobalReport(), 0, 0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);


    ESP_LOGI(TAG, "STOP MQTT");
    ESP_ERROR_CHECK(esp_mqtt_client_stop(mqttClient));
    ESP_LOGI(TAG, "STOP WIFI");
    ESP_ERROR_CHECK(esp_wifi_start());
  }

  // for (int i = 5; i >= 0; i--) {
  //     printf("Restarting in %d seconds...\n", i);
  //     vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
  // printf("Restarting now.\n");

  // fflush(stdout);
  // esp_restart();

  // delete emergency hibernate task
  vTaskDelete(itoTaskHandle);

  hibernate();

}
