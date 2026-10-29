#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <MPU6050.h>

// ================== CONFIG ==================
static const uint8_t  ESPNOW_CHANNEL     = 1;
static const uint32_t PRINT_INTERVAL_MS  = 50;   // 20 Hz
static const uint32_t STALE_MS           = 120;  // freshness threshold (internal)

static const int      BTN_BOOT           = 0;    // GPIO0 (BOOT) active LOW
static const uint32_t DEBOUNCE_MS        = 200;

static const int      BUZZER_PIN         = 5;   // opsional
static const bool     USE_BUZZER         = true;

static const int      LED_PIN            = 16; // NYOBA LED
static const bool     USE_LED            = true;

unsigned long lastBlink = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL = 300; // ms

MPU6050 mpuChest;

// ================== PACKET (MUST MATCH SENDERS) ==================
#pragma pack(push, 1)
struct SensorPacket {
  uint8_t  nodeId;    // 1=wrist, 2=upperarm
  uint32_t seq;
  uint32_t t_ms;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
};
#pragma pack(pop)

// ================== STATE STORAGE ==================
struct NodeState {
  bool     hasData   = false;
  uint32_t lastRxMs  = 0;
  uint32_t lastSeq   = 0;
  uint32_t senderMs  = 0;
  int16_t  ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
};

NodeState n1; // wrist
NodeState n2; // upper arm

bool recording = false;

uint32_t lastPrint = 0;
uint32_t lastBtnMs = 0;
bool lastBtnState  = HIGH;

// ================== WIFI CHANNEL HELPER ==================
static void setWifiChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

// ================== ESPNOW CALLBACK ==================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(SensorPacket)) return;

  SensorPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  uint32_t now = millis();

  if (pkt.nodeId == 1) {
    n1.hasData  = true;
    n1.lastRxMs = now;
    n1.lastSeq  = pkt.seq;
    n1.senderMs = pkt.t_ms;
    n1.ax=pkt.ax; n1.ay=pkt.ay; n1.az=pkt.az;
    n1.gx=pkt.gx; n1.gy=pkt.gy; n1.gz=pkt.gz;
  } else if (pkt.nodeId == 2) {
    n2.hasData  = true;
    n2.lastRxMs = now;
    n2.lastSeq  = pkt.seq;
    n2.senderMs = pkt.t_ms;
    n2.ax=pkt.ax; n2.ay=pkt.ay; n2.az=pkt.az;
    n2.gx=pkt.gx; n2.gy=pkt.gy; n2.gz=pkt.gz;
  }
}

// ================== SETUP ==================
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  setWifiChannel(ESPNOW_CHANNEL);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("# MAC CHEST (STA): %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  Serial.println("# ESP-NOW init...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init GAGAL");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("# ESP-NOW OK, siap terima data");
}

void setupMPUChest() {
  Wire.begin(19, 18); //(21,22) (19, 18)
  mpuChest.initialize();

  Serial.print("# MPU chest: ");
  Serial.println(mpuChest.testConnection() ? "TERDETEKSI" : "TIDAK TERDETEKSI");
}

// Header CSV ringkas (tanpa age)
void printCsvHeader() {
  Serial.println(
    "timestamp,"
    "ax1,ay1,az1,gx1,gy1,gz1,"
    "ax2,ay2,az2,gx2,gy2,gz2,"
    "ax3,ay3,az3,gx3,gy3,gz3"
  );
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BTN_BOOT, INPUT_PULLUP);

  if (USE_BUZZER) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  if (USE_LED) {
    pinMode (LED_PIN, OUTPUT);
    digitalWrite (LED_PIN, LOW);
  }

  Serial.println("\n=== RECEIVER CHEST (CSV RINGKAS) ===");
  setupEspNow();
  setupMPUChest();

  Serial.println("# Tekan BOOT untuk START/STOP logging.");
  Serial.println("# Status awal: RECORDING = OFF");

  lastPrint = millis();
}

// ================== BUTTON HANDLER ==================
void handleBootToggle() {
  bool btn = digitalRead(BTN_BOOT); // LOW kalau ditekan
  uint32_t now = millis();

  if (lastBtnState == HIGH && btn == LOW) {
    if (now - lastBtnMs > DEBOUNCE_MS) {
      lastBtnMs = now;
      recording = !recording;

      if (recording) {
        Serial.println("# RECORDING = ON");
        printCsvHeader();
      } else {
        Serial.println("# RECORDING = OFF");
      }
    }
  }
  lastBtnState = btn;
}

// ================== LOOP ==================
void loop() {
  handleBootToggle();

  uint32_t now = millis();
  if ((uint32_t)(now - lastPrint) >= PRINT_INTERVAL_MS) {
    lastPrint += PRINT_INTERVAL_MS;

    // baca MPU chest (node 3)
    int16_t ax3, ay3, az3, gx3, gy3, gz3;
    mpuChest.getAcceleration(&ax3, &ay3, &az3);
    mpuChest.getRotation(&gx3, &gy3, &gz3);

    // age dihitung internal (tidak dicetak)
    int32_t age1 = n1.hasData ? (int32_t)(now - n1.lastRxMs) : -1;
    int32_t age2 = n2.hasData ? (int32_t)(now - n2.lastRxMs) : -1;

    // buzzer warning jika data node 1/2 stale / belum ada
    if (USE_BUZZER) {
      bool stale = (age1 < 0) || (age2 < 0) ||
                   (age1 > (int32_t)STALE_MS) || (age2 > (int32_t)STALE_MS);
      digitalWrite(BUZZER_PIN, stale ? HIGH : LOW);
    }

    if (USE_LED) {
      bool stale = (age1 < 0) || (age2 < 0) ||
                   (age1 > (int32_t)STALE_MS) || (age2 > (int32_t)STALE_MS);
      //digitalWrite(LED_PIN, stale ? HIGH : LOW);

      if (stale) {
        unsigned long nowMs = millis();
        if (nowMs - lastBlink >= BLINK_INTERVAL) {
          lastBlink = nowMs;
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState);
        }
      } 
      else {
        digitalWrite(LED_PIN, LOW); // normal → LED mati
        ledState = false;
      }
    }

    if (!recording) return;

    // ambil data terakhir node 1/2
    int16_t ax1=n1.ax, ay1=n1.ay, az1=n1.az, gx1=n1.gx, gy1=n1.gy, gz1=n1.gz;
    int16_t ax2=n2.ax, ay2=n2.ay, az2=n2.az, gx2=n2.gx, gy2=n2.gy, gz2=n2.gz;

    if (!n1.hasData) ax1=ay1=az1=gx1=gy1=gz1=0;
    if (!n2.hasData) ax2=ay2=az2=gx2=gy2=gz2=0;

    // CSV ringkas
    Serial.printf(
      "%lu,"
      "%d,%d,%d,%d,%d,%d,"
      "%d,%d,%d,%d,%d,%d,"
      "%d,%d,%d,%d,%d,%d\n",
      (unsigned long)now,
      ax1,ay1,az1,gx1,gy1,gz1,
      ax2,ay2,az2,gx2,gy2,gz2,
      ax3,ay3,az3,gx3,gy3,gz3
    );
  }
}