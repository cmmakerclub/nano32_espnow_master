#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <CMMC_NB_IoT.h>
#include "data_type.h"

HardwareSerial Serial1(2);
CMMC_PACKET_T pArr[60];
int pArrIdx = 0;
char espnowMsg[300];

#define rxPin (16)
#define txPin (17)

#define BOARD_LED (25)
#define GREEN_LED (23)
#define RED_LED (22)

#define RESET_PIN (13)

#define LED_PIN

CMMC_NB_IoT nb(&Serial1);
uint32_t counter = 0;
uint32_t sentCnt = 0;

void str2Hex(const char* text, char* buffer);
void toHexString(const uint8_t array[], size_t len, char buffer[]);

uint8_t currentSleepTimeMinuteByte = 5;
uint32_t msAfterESPNowRecv = millis();

bool dirty = false;
bool isNbConnected = false;

// String token = "b98ce7b0-185b-11e8-8630-b33e669e2295";
// String token = "f169c8e0-a872-11e8-8e2c-19a3b7904cb9";
String token = "3ffbfb30-aaae-11e8-8e2c-19a3b7904cb9";
char tokenHex[100];
uint32_t prev;
// uint8_t remoteMac[6] = {0x2e, 0x3a, 0xe8, 0x12, 0xbe, 0x92};
esp_now_peer_info_t slave;

void setup() {
  bzero(&slave, sizeof(slave));
  Serial.begin(115200);
  Serial.println("HELLO..");
  Serial1.begin(9600, SERIAL_8N1, rxPin, txPin, false);
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


  Serial.println("Serial1..");
  WiFi.disconnect();
  Serial.println("[1]..");
  Serial.println("[1.1]..");
  WiFi.mode(WIFI_AP_STA);
  Serial.println("[1.2]..");
  delay(200);
  Serial.println("[2]..");
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());

  str2Hex(token.c_str(), tokenHex);
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
  }); nb.onDeviceReady([]() {
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
    Serial.println(F("[user] Device rebooted."));
    Serial.println("Connecting to NB-IoT...");
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
    delay(300);
  });

  nb.onConnected([]() {
    ledcWrite(1, 20);
    Serial.println(F("[user] Device rebooted."));
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    ledcWrite(1, 20);
    Serial.print("[user] NB-IoT Network connected at (");
    Serial.print(millis());
    Serial.println("ms)");
    delay(3000);
    Serial.println(nb.createUdpSocket("103.20.205.85", 5683, UDPConfig::ENABLE_RECV));
    Serial.println(nb.createUdpSocket("103.212.181.167", 55566, UDPConfig::ENABLE_RECV));
    isNbConnected = 1;
    delay(1000);
  });

  Serial.println("WAIT... 2s");
  delay(2000);
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

uint32_t lastSentOkMillis;
unsigned int ct = 1;

void loop() {
  if ( (millis() - lastSentOkMillis) > 10800 * 1000) {
    ESP.deepSleep(1e6);
  }
  nb.loop();
  if ( (millis() - msAfterESPNowRecv) > 500 && (pArrIdx > 0) && (isNbConnected)) {
    static char buffer[600];
    bzero(buffer, sizeof(buffer));
    static char b[300];
    static char msgId[5];
    Serial.printf("pArrIdx = %d\r\n", pArrIdx);
    Serial.println();
    Serial.println(String(ct, HEX));;
    for (int i = pArrIdx - 1; i >= 0; i--) {
      digitalWrite(RED_LED, !digitalRead(RED_LED));
      Serial.printf("reading idx = %d\r\n", i);
      toHexString((uint8_t*)  &pArr[i], sizeof(CMMC_PACKET_T), (char*)espnowMsg);
      sprintf(b, "{\"ct\":\"%lu\", \"sleep\":\"%lu\", \"payload\": \"%s\"}", ct++, currentSleepTimeMinuteByte, espnowMsg);
      str2Hex(b, buffer);
      sprintf(msgId, "%04lu", ct);
      Serial.printf("msgId = %s\r\n", msgId);
      String p3 = "";
      p3 += String("40");
      p3 += String("02");
      // p3 += String(ct, HEX);
      p3 += String(msgId);
      p3 += String("b5");
      p3 += String("4e42496f54"); // NB-IoT
      p3 += String("0d");
      p3 += String("17");
      p3 +=  String(tokenHex);
      p3 += String("ff");
      p3 += String(buffer);
      Serial.println(p3);
      int rt = 0;
      while (true) {
        ledcWrite(1, 20);
        if (nb.sendMessageHex(p3.c_str(), 0)) { 
          delay(300); 
          ledcWrite(1, 0);

          ledcWrite(1, 20);
          nb.sendMessageHex(p3.c_str(), 0);
          delay(300);
          ledcWrite(1, 0);

          ledcWrite(1, 20);
          nb.sendMessageHex(p3.c_str(), 0);
          delay(300);
          ledcWrite(1, 0);

          Serial.println(">> [ais] socket0: send ok.");
          pArrIdx--;
          lastSentOkMillis = millis();
          delay(1000);
          break;
        }
        else {
          Serial.println(">> [ais] socket0: send failed.");
          if (++rt > 5) {
            delay(100);
            // ESP.restart();
            ESP.deepSleep(1e6);
            delay(100);
            break;
          }
        }
        delay(100);
      }
    }
    digitalWrite(RED_LED, LOW);
    prev = millis();
  }
  // if (millis() - prev > 1000) {
  //   prev = millis();
  //   Serial.printf("rate: recv=%lu, sent=%lu hz\r", counter, sentCnt);
  //   counter = 0;
  //   sentCnt = 0;
  // }
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