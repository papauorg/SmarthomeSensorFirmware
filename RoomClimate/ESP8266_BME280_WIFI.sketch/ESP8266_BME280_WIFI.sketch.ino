#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <ESP8266WiFi.h>

const char* WIFI_SSID   = "";
const char* WIFI_PASSWD = "";

const int SLEEPTIME_IN_SECONDS = 30;

const char* OPENHAB_SERVER = "openhab.local";
const int OPENHAB_PORT = 443;


// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct {
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t bssid[6]; // 6 bytes, 11 in total
  uint8_t padding; // 12 bytes
  //uint8_t ip[4];    // 4 bytes, 15 in total
  //uint8_t netmask;  // 1 byte,  16 in total
  //uint8_t dns_ip[4];// 4 bytes, 20 in total
  //uint8_t gateway[4];// 4 bytes,24 in total
} rtcData;

uint8_t deviceMac[6];

Adafruit_BME280 bme;

typedef struct{
  float temperature;
  float humidity;
}bmeSensorValues;

void setup() {
  Serial.begin(115200);
  setWifiEnergySavingSettings();
  
  delay(100);

  Serial.println("Waking up");
  
  bool bmeStatus = initBmeSensor();

  if (bmeStatus){
    bmeSensorValues values = readBmeValues();
    
    connectToWifi();
    sendSensorValues(values);
    disconnectWifi();
  }

  goToDeepSleep();
}

void loop() {
}

bool initBmeSensor(){
  bool status;
  status = bme.begin(0x76);  
  if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while(1) {}
  }
  return status;
}

bmeSensorValues readBmeValues(){

  bmeSensorValues values;
  values.temperature = bme.readTemperature();
  values.humidity = bme.readHumidity();
  Serial.print("Temperature: ");
  Serial.println(values.temperature);
  Serial.print("Humidity: ");
  Serial.println(values.humidity);

  return values;
}

bool readWifiInfoFromRtc()
{
  Serial.println("Read Wifi info from RTC memory");
  bool rtcValid = false;
  if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if (crc == rtcData.crc32)
    {
      rtcValid = true;
    }
  }

  return rtcValid;
}

void writeWifiInfoToRtc()
{
  rtcData.channel = WiFi.channel();
  memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
}

void connectToWifi(){
  // Try to read WiFi settings and IPs from RTC memory
  bool rtcValid = readWifiInfoFromRtc();
  
  // bring up wifi connection
  WiFi.forceSleepWake();
  delay( 1 );
  WiFi.persistent( false );
  WiFi.mode( WIFI_STA );

  if (rtcValid == true)
  {
    Serial.println("Reusing connection info from rtc data");
    WiFi.begin(WIFI_SSID, WIFI_PASSWD, rtcData.channel, rtcData.bssid, true);
  }
  else 
  {
    // when no connection info is read, use
    // network scanning to retrieve
    // the info from the AP once
    Serial.println("Connecting via nework scan");
    WiFi.begin( WIFI_SSID, WIFI_PASSWD );
  }

  int retries = 0;
  int wifiStatus = WiFi.status();
  while (wifiStatus != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (retries == 20)
    {
      Serial.println("Could not connect to wifi. Resetting and trying again later.");
      disconnectWifi();
      resetWifiInfos();
      goToDeepSleep();
      return;
    }
    wifiStatus = WiFi.status();
    retries += 1;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Netmask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  
  WiFi.macAddress(deviceMac);
  Serial.print("Device MAC: ");
  for(int i = 0; i < sizeof(deviceMac); ++i)
  {
    Serial.print(deviceMac[i], 16); 
  }
  Serial.println();

  // Write current connection info back to RTC
  writeWifiInfoToRtc();
}

void sendSensorValues(bmeSensorValues values){
  sendHttpRequest("/rest/items/temperature", values.temperature);
  sendHttpRequest("/rest/items/humidity", values.humidity);
}

void sendHttpRequest(char* baseUrl, float value){
  Serial.print("Try to send HTTP request to ");
  Serial.print(OPENHAB_SERVER);
  Serial.print(":");
  Serial.print(OPENHAB_PORT);
  Serial.print(baseUrl);
  for(int i = 0; i < sizeof(deviceMac); ++i)
  {
    Serial.print(deviceMac[i], 16); 
  }
  Serial.println();
  
  WiFiClientSecure client;
  client.setInsecure(); // do not validate self signed server certificate
  
  if (client.connect(OPENHAB_SERVER, OPENHAB_PORT)) {
    Serial.println("Connected, sending request...");
    client.print("POST "); client.print(baseUrl); 
    // append mac address of sensor
    for(int i = 0; i < sizeof(deviceMac); ++i)
    {
      client.print(deviceMac[i], 16); 
    }
    client.println(" HTTP/1.1");
    
    client.print("Host: "); client.println(OPENHAB_SERVER);
    client.println("Accept: application/json");
    client.println("Content-Type: text/plain");
    if (value >= 10 || value <= -10){
      client.println("Content-Length: 4");
    } else {
      client.println("Content-Length: 3");
    }
    client.println();
    client.println(value, 1);
    client.println();
  } 
  else {
    Serial.println("Failed to send HTTP request.");
  }
}

void disconnectWifi(){
  WiFi.disconnect( true );
  delay( 1 );
}

void goToDeepSleep(){
  Serial.print("Going to deep sleep for ");
  Serial.print(SLEEPTIME_IN_SECONDS);
  Serial.println(" seconds");
  ESP.deepSleep( SLEEPTIME_IN_SECONDS * 1000000, WAKE_RF_DISABLED );
}

void setWifiEnergySavingSettings(){
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
}

void resetWifiInfos()
{
  rtcData.crc32 = 0; // set invalid crc sum so next run it will be invalid.
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
}

uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while( length-- ) {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}
