/* ESP8266 AWS for Epever Tracker modbus telemetry
 *  
 * Author: Ironfall, this includes code from Anthony Elder 
 * License: GPLv3
 */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>

extern "C" {
#include "libb64/cdecode.h"
}

const char* ssid = "[your wifi ssid]";
const char* password = "[your wifi password]";

// Find this awsEndpoint in the AWS Console: Manage - Things, choose your thing
// choose Interact, its the HTTPS Rest endpoint 
const char* awsEndpoint = "[your aws iot endpoint";

// For the two certificate strings below paste in the text of your AWS 
// device certificate and private key, comment out the BEGIN and END 
// lines, add a quote character at the start of each line and a quote 
// and backslash at the end of each line:

// xxxxxxxxxx-certificate.pem.crt
const String certificatePemCrt = \
//-----BEGIN CERTIFICATE-----
//[your certificare from AWS iot]
//-----END CERTIFICATE-----


// xxxxxxxxxx-private.pem.key
const String privatePemKey = \
//-----BEGIN RSA PRIVATE KEY-----
//[your certificare from AWS iot]
//-----END RSA PRIVATE KEY-----



// This is the AWS IoT CA Certificate from: 
// https://docs.aws.amazon.com/iot/latest/developerguide/managing-device-certs.html#server-authentication
// This one in here is the 'RSA 2048 bit key: Amazon Root CA 1' which is valid 
// until January 16, 2038 so unless it gets revoked you can leave this as is:
const String caPemCrt = \
//-----BEGIN CERTIFICATE-----
//[the aws iot ca certificare]
//-----END CERTIFICATE-----

WiFiClientSecure wiFiClient;
void msgReceived(char* topic, byte* payload, unsigned int len);
PubSubClient pubSubClient(awsEndpoint, 8883, msgReceived, wiFiClient); 

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

//these are pins on the esp8266 connected to the mobus adapter
#define MAX485_DE D5
#define MAX485_RE_NEG D6

ModbusMaster node;
SimpleTimer timer;

int timerTask1, timerTask2, timerTask3;


unsigned long lastPublish;
int msgCount;

float batVolt;
float batPercentage = 0.0;

int outputValue1 = 0;
int countIt = 0;

void msgReceived(char* topic, byte* payload, unsigned int length) {
  Serial1.print("Message received on "); 
  Serial1.print(topic); Serial1.print(": ");
  for (int i = 0; i < length; i++) {
    Serial1.print((char)payload[i]);
  }
  Serial1.println();
}

void pubSubCheckConnect() {
  if ( ! pubSubClient.connected()) {
    Serial1.print("PubSubClient connecting to: "); Serial1.print(awsEndpoint);
    while ( ! pubSubClient.connected()) {
      Serial1.print(".");
      pubSubClient.connect("ESPthing");
    }
    Serial1.println(" connected");
    pubSubClient.subscribe("inTopic");
  }
  pubSubClient.loop();
}

int b64decode(String b64Text, uint8_t* output) {
  base64_decodestate s;
  base64_init_decodestate(&s);
  int cnt = base64_decode_block(b64Text.c_str(), b64Text.length(), (char*)output, &s);
  return cnt;
}

void setCurrentTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial1.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial1.print(".");
    now = time(nullptr);
  }
  Serial1.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial1.print("Current time: "); 
  Serial1.print(asctime(&timeinfo));
}

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  //shorter delays result in strange behavior for me with a epever tracker Tracer6415AN
  delay(4000);
  Serial.begin(115200);
  Serial.swap();
 
  Serial1.begin(115200);
  Serial1.println();
  Serial1.println("ESP8266 solar data collector");
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus slave ID 1 per epever docs http://www.solar-elektro.cz/data/dokumenty/1733_modbus_protocol.pdf
  node.begin(1, Serial);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial1.print("Connecting to "); 
  Serial1.print(ssid);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();
  Serial1.print(", WiFi connected, IP address: "); 
  Serial1.println(WiFi.localIP());

  setCurrentTime();

  uint8_t binaryCert[certificatePemCrt.length() * 3 / 4];
  int len = b64decode(certificatePemCrt, binaryCert);
  wiFiClient.setCertificate(binaryCert, len);
  
  uint8_t binaryPrivate[privatePemKey.length() * 3 / 4];
  len = b64decode(privatePemKey, binaryPrivate);
  wiFiClient.setPrivateKey(binaryPrivate, len);

  uint8_t binaryCA[caPemCrt.length() * 3 / 4];
  len = b64decode(caPemCrt, binaryCA);
  wiFiClient.setCACert(binaryCA, len);
  
}

  int counter = 0;
  void loop() {
  if (counter > 300) {
    setCurrentTime();
    counter = 0;
  }
  const int capacity = JSON_OBJECT_SIZE(19);
  StaticJsonDocument<capacity> doc;

  pubSubCheckConnect();
  static char outstr[15];
  uint8_t result;
  uint16_t data[6];
  result = node.writeSingleCoil(0x0002, true);
  delay(50);
  result = node.readInputRegisters(0x3100, 16);

  if (result == node.ku8MBSuccess) {

    // grab all the useful data points
    Serial1.println("");
    Serial1.println("Tracer CC Data:");
    
    Serial1.print("PV Voltage: ");                  Serial1.println(node.getResponseBuffer(0x0)/100.0f);
    doc["pv_v"] = dtostrf(node.getResponseBuffer(0x0)/100.0f,5,2,outstr);
    Serial1.print("PV Current: ");                  Serial1.println(node.getResponseBuffer(0x01)/100.0f);
    doc["pv_c"] = dtostrf(node.getResponseBuffer(0x01)/100.0f,5,2,outstr);
    Serial1.print("PV Power L: ");                  Serial1.println(node.getResponseBuffer(0x02)/100.0f);
    doc["pv_pl"] = dtostrf(node.getResponseBuffer(0x02)/100.0f,5,2,outstr);
    Serial1.print("PV Power H: ");                  Serial1.println(node.getResponseBuffer(0x03)/100.0f);
    doc["pv_ph"] = dtostrf(node.getResponseBuffer(0x03)/100.0f,5,2,outstr);
    Serial1.print("Vbatt: ");                       Serial1.println(node.getResponseBuffer(0x04)/100.0f);
    doc["b_v"] = dtostrf(node.getResponseBuffer(0x04)/100.0f,5,2,outstr);
    Serial1.print("Bat charge cur: ");              Serial1.println(node.getResponseBuffer(0x05)/100.0f);
    doc["b_c_c"] = dtostrf(node.getResponseBuffer(0x05)/100.0f,5,2,outstr);
    Serial1.print("Battery charging power L: ");    Serial1.println(node.getResponseBuffer(0x06)/100.0f);
    doc["b_cpl"] = dtostrf(node.getResponseBuffer(0x06)/100.0f,5,2,outstr);
    Serial1.print("Battery charging power H: ");    Serial1.println(node.getResponseBuffer(0x07)/100.0f);
    doc["b_cph"] = dtostrf(node.getResponseBuffer(0x07)/100.0f,5,2,outstr);
    Serial1.print("load voltage: ");                Serial1.println(node.getResponseBuffer(0x0C)/100.0f);
    doc["l_v"] = dtostrf(node.getResponseBuffer(0x0C)/100.0f,5,2,outstr);
    Serial1.print("load current: ");                Serial1.println(node.getResponseBuffer(0x0D)/100.0f);
    doc["l_c"] = dtostrf(node.getResponseBuffer(0x0D)/100.0f,5,2,outstr);
    Serial1.print("load power: ");                  Serial1.println(node.getResponseBuffer(0x0E)/100.0f);
    doc["l_p"] = dtostrf(node.getResponseBuffer(0x0E)/100.0f,5,2,outstr);
    Serial1.print("alt current & power: ");          Serial1.println((node.getResponseBuffer(0x0D) + node.getResponseBuffer(0x0E) << 16)/100.0f);

  }
  else {
    Serial1.println("result failed3100");
    pubSubClient.publish("result", "{\"3100\":\"fail\"}");
  }

  result = node.readInputRegisters(0x311A, 2);
  if (result == node.ku8MBSuccess)  {
   Serial1.print("Bat percent: ");                     Serial1.println(node.getResponseBuffer(0x00)/1.0f);
   doc["b_pct"] = dtostrf(node.getResponseBuffer(0x00)/1.0f,5,2,outstr);
   Serial1.print("Battery Temp: ");           Serial1.println(node.getResponseBuffer(0x01)/100.0f);
   doc["b_tmp"] = dtostrf(node.getResponseBuffer(0x01)/100.0f,5,2,outstr);
  }
  else {
    Serial1.println("result failed 3110");
    pubSubClient.publish("result", "{\"3110\":\"fail\"}");
  }

  result = node.readInputRegisters(0x3300, 16);
  if (result == node.ku8MBSuccess)  {
   Serial1.print("Max PV input voltage today: ");   Serial1.println(node.getResponseBuffer(0x00)/100.0f);
   doc["pv_mx_v"] = dtostrf(node.getResponseBuffer(0x00)/100.0f,5,2,outstr);
   Serial1.print("Min PV input voltage today: ");  Serial1.println(node.getResponseBuffer(0x01)/100.0f);
   doc["pv_mn_v"] = dtostrf(node.getResponseBuffer(0x01)/100.0f,5,2,outstr);
   Serial1.print("Max battery voltage today: ");  Serial1.println(node.getResponseBuffer(0x02)/100.0f);
   doc["b_mx_v"] = dtostrf(node.getResponseBuffer(0x02)/100.0f,5,2,outstr);
   Serial1.print("Min bat input voltage today: "); Serial1.println(node.getResponseBuffer(0x03)/100.0f);
   doc["b_mn_v"] = dtostrf(node.getResponseBuffer(0x03)/100.0f,5,2,outstr);
 
  }
  else {
    Serial1.println("result failed 3300");
    pubSubClient.publish("result", "{\"3300\":\"fail\"}");
  }
  
  char buffer[256];
  serializeJson(doc, buffer);
   
  Serial1.print("strlen: ");
  Serial1.println(strlen(buffer));
  if (strlen(buffer) > 5) {
    pubSubClient.publish("solar/data", buffer);
    Serial1.print("published data: ");
    Serial1.println(buffer);
  }
  else {
    Serial1.println("Did NOT publish, strlen < 5");
  }
  doc.clear();
  counter++;
  // trying to sample faster than this seems to cause unreliability with the Tracer6415AN
  delay(30000);
  
}
