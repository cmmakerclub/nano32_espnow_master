#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <CMMC_NB_IoT.h>
#include "data_type.h"
#include "coap.h"
#include "coap-helper.h"

// #include "soc/soc.h"
// #include "soc/rtc_cntl_reg.h"

HardwareSerial mySerial1(2);
CMMC_PACKET_T pArr[60];
int pArrIdx = 0;
char espnowMsg[300];

#define rxPin (16)
#define txPin (17)

#define ANALOG_PIN_0 (36)

#define BOARD_LED (25)
#define GREEN_LED (23)
#define RED_LED (22)

#define RESET_PIN (13)

#define LED_PIN

CMMC_NB_IoT nb(&mySerial1);
uint32_t counter = 0;
uint32_t sentCnt = 0;

void str2Hex(const char* text, char* buffer);
void toHexString(const uint8_t array[], size_t len, char buffer[]);

uint8_t currentSleepTimeMinuteByte = 5;

uint32_t msAfterESPNowRecv = millis();

bool dirty = false;
bool isNbConnected = false;

// String token = "3ffbfb30-aaae-11e8-8e2c-19a3b7904cb9";
char tokenHex[100];
uint32_t prev;
// uint8_t remoteMac[6] = {0x2e, 0x3a, 0xe8, 0x12, 0xbe, 0x92};
esp_now_peer_info_t slave;

CoapPacket p;

void setup() {
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  bzero(&slave, sizeof(slave));
  Serial.begin(115200);
  mySerial1.begin(9600);
  Serial.println("HELLO..");
  analogReadResolution(10); // 10Bit resolution
  analogSetAttenuation(ADC_2_5db);  // 0=0db (0..1V) 1= 2,5dB; 2=-6dB (0..2V); 3=-11dB
  // Serial1.begin(9600, SERIAL_8N1, rxPin, txPin, false);
  // Serial1.begin(9600);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);
  digitalWrite(RESET_PIN, LOW);
  delay(100); 
  ledcAttachPin(BOARD_LED, 1);

  ledcSetup(1, 12000, 8);
  ledcWrite(1, 20);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH); 

  Serial.println("");
  Serial.println("/COAP PACKET..");
  WiFi.disconnect(); 
  WiFi.mode(WIFI_AP_STA);
  delay(200);
  Serial.println("[2]..");
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());

  // str2Hex(token.c_str(), tokenHex);
  Serial.println(tokenHex);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  delay(100); 
  nb.setDebugStream(&Serial); 
  nb.onDeviceReboot([]() {
    ledcWrite(1, 0);
    Serial.println(F("[user] Device rebooted."));
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    delay(2000);
  }); 
  
  nb.onDeviceReady([]() {
    Serial.println("[user] Device Ready!");
  });

  nb.onDeviceInfo([](CMMC_NB_IoT::DeviceInfo device) {
    Serial.print(F("# Module IMEI-->  "));
    Serial.println(device.imei);
    Serial.print(F("# Firmware ver-->  "));
    Serial.println(device.firmware);
    Serial.print(F("# IMSI SIM-->  "));
    Serial.println(device.imsi);
  });

  nb.onMessageArrived([](char *text, size_t len, uint8_t socketId, char* ip, uint16_t port) {
    char buffer[100];
    sprintf(buffer, "++ [recv:] socketId=%u, ip=%s, port=%u, len=%d bytes (%lums)", socketId, ip, port, len, millis());
    Serial.println(buffer);
  });

  nb.onConnecting([]() {
    ledcWrite(1, 0);
    delay(300);
    ledcWrite(1, 20);
    Serial.println("Connecting to NB-IoT...");
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
    delay(300);
  });

  nb.onConnected([]() {
    ledcWrite(1, 0);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    Serial.print("[user] NB-IoT Network connected at (");
    Serial.print(millis());
    Serial.println("ms)");
    delay(3000);
    Serial.println(nb.createUdpSocket("103.20.205.85", 5683, UDPConfig::ENABLE_RECV));
    // Serial.println(nb.createUdpSocket("103.212.181.167", 55566, UDPConfig::ENABLE_RECV));
    isNbConnected = 1;
    delay(1000);
  });

  Serial.println("WAIT... 2s");
  delay(2000);
  Serial.println("Rebooting module"); 
  
  nb.hello(); 
  nb.rebootModule(); 

  esp_now_register_send_cb([&] (const uint8_t *mac_addr, esp_now_send_status_t status) {
    sentCnt++;
  });

  esp_now_register_recv_cb([&](const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
    memcpy(&slave.peer_addr, mac_addr, 6);
    CMMC_PACKET_T wrapped;
    CMMC_SENSOR_DATA_T packet;

    memcpy(&packet, data, sizeof(packet));
    memcpy(&wrapped.data, &packet, sizeof(packet));
    wrapped.sleepTime = (uint32_t)currentSleepTimeMinuteByte;
    // packet.field9 = counter;
    wrapped.ms = millis();
    wrapped.sum = counter;

    pArr[pArrIdx] = wrapped;
    pArrIdx = (pArrIdx + 1) % 30;

    esp_now_send(mac_addr, &currentSleepTimeMinuteByte, 1);
    Serial.printf("(uint32_t) sleepTime = %lu\r\n", (uint32_t) currentSleepTimeMinuteByte);
    for (int i = 0 ; i < sizeof(wrapped); i++) {
      Serial.printf("%02x", ((uint8_t*)&wrapped)[i]);
    }
    Serial.println();

    esp_err_t addStatus = esp_now_add_peer(&slave);
    uint8_t time = 1;
    // esp_err_t result = esp_now_send(mac_addr, &time, 1);
    counter++;

    msAfterESPNowRecv = millis();
  });

  prev = millis();

}

uint32_t lastSentOkMillis = millis();
unsigned int ct = 1; 
void sendPacket(const char *text, int buflen) ;

void generatePacket() {
    static char buffer[600];
    bzero(buffer, sizeof(buffer));
    static char b[300];
    static char msgId[5];
    Serial.printf("pArrIdx=%d, ct=%d\r\n", pArrIdx, ct);
    int analogValue = analogRead(ANALOG_PIN_0);
    float batt = map(analogValue, 0, 1024, 0, 134); // max=1.34v
    float batt_percent = map(batt, 0, 116, 0, 100); // devided to 1.16v
    sprintf(b, "{\"a0\":%d,\"batt\":%s,\"batt_p\":%s,\"ct\":%lu,\"sleep\":%lu,\"payload\":\"%s\"}", analogValue, 
          String(batt/100).c_str(), String(batt_percent).c_str(), ct++, currentSleepTimeMinuteByte, espnowMsg);
    Serial.println(b);

    IPAddress ip = IPAddress(103,20,205,85);
    uint8_t _buffer[BUF_MAX_SIZE];
    char myb[BUF_MAX_SIZE];
    bzero(_buffer, sizeof(_buffer));
    uint16_t buflen = send(_buffer, ip, 5683, "NBIoT/9ee9d8a0-c657-11e8-8443-17f06f0c0a93", COAP_CON, COAP_POST, NULL, 0, (uint8_t*) b, strlen(b)); 
    // for (int x = 0; x < buflen; x++) {
    //   Serial.printf("%02x", _buffer[x]); 
    // } 
    // Serial.println(); 
    
    if (pArrIdx > 0) {
      for (int i = pArrIdx - 1; i >= 0; i--) {
        digitalWrite(RED_LED, !digitalRead(RED_LED));
        Serial.printf("reading idx = %d\r\n", i);
        toHexString((uint8_t*)  &pArr[i], sizeof(CMMC_PACKET_T), (char*)espnowMsg);
        // Serial.println(b);
        // str2Hex(b, buffer);
        // sprintf(msgId, "%04lu", ct);
        // Serial.printf("msgId = %s\r\n", msgId);
        // String p3 = "";
        // sendPacket(p3.c_str());
      } 
    }
    else {
      memcpy(myb, _buffer, buflen);
      sendPacket(myb, buflen);
    }
}

void sendPacket(const char *text, int buflen) {
    int rt = 0;
    while (true) {
      ledcWrite(1, 50); 
      if (nb.sendMessageHex(text, buflen, 0)) { 
        ledcWrite(1, 0); 
        Serial.println(">> [ais] socket0: send ok.");
        lastSentOkMillis = millis();
        delay(100);
        break;
      }
      else {
        Serial.println(">> [ais] socket0: send failed.");
        if (++rt > 5) {
          delay(100);
          ESP.deepSleep(1e6);
          delay(100);
          break;
        }
      }
      delay(200);
    } 
}

void loop() {
  nb.loop();

  if ( (millis() - lastSentOkMillis) > 10800 * 1000) {
    ESP.deepSleep(1e6);
  }


  float MINUTE = 0.1;
  int ms = MINUTE *  60 * 1000;
  if ( isNbConnected &&  (millis() - lastSentOkMillis > ms) )  {
    Serial.printf("KEEP ALIVE.. %d>%d\r\n", (millis() - lastSentOkMillis), ms);
    generatePacket();
    lastSentOkMillis = millis();
  }

  if ( (millis() - msAfterESPNowRecv) > 500 && (pArrIdx > 0) && (isNbConnected)) {
    generatePacket();
    digitalWrite(RED_LED, LOW);
    prev = millis();
  }

} 

void str2Hex(const char* text, char* buffer) {
  size_t len = strlen(text);
  for (int i = 0 ; i < len; i++) {
    sprintf(buffer + i * 2, "%02x", text[i]);
  }
}

void toHexString(const uint8_t array[], size_t len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}