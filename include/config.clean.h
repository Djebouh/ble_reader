// sleep between to runs in seconds
const int     SLEEP_DURATION = 30 * 60;
// emergency hibernate countdown in seconds
const int     ITO_SERVICE = 7 * 60;
// emergency hibernate countdown in seconds
const int     ITO_APP = ITO_SERVICE + 60;
//  number of attemps to connect and extra data for a given device
const int     RETRY = 3;

// MQTT server where to report
const char    MQTT_HOST[]="MY_IP";
const char    MQTT_URI[]="mqtt://MY_IP:MY_PORT";
const int     MQTT_PORT=MY_PORT;
