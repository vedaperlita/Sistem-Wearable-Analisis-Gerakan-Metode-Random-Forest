#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <MPU6050.h>

static const uint8_t NODE_ID = 2;          // 1 = wrist
static const uint8_t ESPNOW_CHANNEL = 1;   // samakan di semua node
static const uint32_t SAMPLE_INTERVAL_MS = 50; // 20 Hz

// === GANTI INI: MAC ESP32 DADA (Receiver) ===
uint8_t RECEIVER_MAC[6] = {0x60, 0x55, 0xF9, 0x71, 0x62, 0xB0};

MPU6050 mpu;

#pragma pack(push, 1)
struct SensorPacket {
  uint8_t  nodeId;
  uint32_t seq;
  uint32_t t_ms;      // timestamp lokal sender (millis) (buat debugging/analisis)
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
};
#pragma pack(pop)

volatile uint32_t g_seq = 0;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // debug kalau perlu:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[SEND] OK" : "[SEND] FAIL");
}

static void setWifiChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

void setupMPU() {
  Wire.begin(19, 18); // SDA=21, SCL=22
  mpu.initialize();

  Serial.println("[WRIST] MPU6050 TERDETEKSI ✔");
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  setWifiChannel(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[WRIST] ESP-NOW init GAGAL");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[WRIST] Gagal menambah peer receiver");
  } else {
    Serial.println("[WRIST] Peer receiver OK");
  }
}

uint32_t lastTick = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SENDER WRIST START ===");

  setupMPU();
  setupEspNow();

  lastTick = millis();
}

void loop() {
  const uint32_t now = millis();

  if ((uint32_t)(now - lastTick) >= SAMPLE_INTERVAL_MS) {
    lastTick += SAMPLE_INTERVAL_MS; // menjaga interval stabil

    SensorPacket pkt;
    pkt.nodeId = NODE_ID;
    pkt.seq = ++g_seq;
    pkt.t_ms = now;

    mpu.getAcceleration(&pkt.ax, &pkt.ay, &pkt.az);
    mpu.getRotation(&pkt.gx, &pkt.gy, &pkt.gz);

    esp_now_send(RECEIVER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  }
}
