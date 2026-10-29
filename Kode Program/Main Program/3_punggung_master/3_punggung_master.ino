#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>
#include "rf_model.h"

#define GYRO_FACTOR 3752.9

// =========================
// CONFIG
// =========================
#define BUZZER_PIN 5

Adafruit_MPU6050 mpu;
Eloquent::ML::Port::RandomForest rf;

// Min-max punggung (sesuai dataset terbaru)
float gx3_min=-18914, gx3_max=14162;
float gy3_min=-24746, gy3_max=32697;
float gz3_min=-6781,  gz3_max=4520;

float normalize(float val, float minVal, float maxVal) {
    return (val - minVal) / (maxVal - minVal);
}

// =========================
// STRUCT DATA
// =========================
typedef struct struct_message {
    int id;
    float features[7];
} struct_message;

// =========================
// VARIABLE DATA
// =========================
float pergelangan_features[7] = {0};
float lenganAtas_features[7] = {0};

bool pergelanganReceived = false;
bool lenganAtasReceived = false;

// =========================
// COUNTER PENGUJIAN
// =========================
int totalBenar = 0;
int totalSalah = 0;
int totalPercobaan = 0;

const int TARGET_REPETISI = 20;

// =========================
// FEATURE EXTRACTION
// =========================
void extractFeature(float arr[], int N, float output[7]) {

    float sum = 0;
    double energy = 0;

    float maxVal = arr[0];
    float minVal = arr[0];

    for (int i = 0; i < N; i++) {
        sum += arr[i];
        energy += (double)arr[i] * (double)arr[i];

        if (arr[i] > maxVal) maxVal = arr[i];
        if (arr[i] < minVal) minVal = arr[i];
    }

    float mean = sum / N;

    float std_val = 0;
    for (int i = 0; i < N; i++) {
        std_val += (arr[i] - mean) * (arr[i] - mean);
    }
    std_val = sqrt(std_val / N);

    float sorted[40];
    memcpy(sorted, arr, sizeof(float) * N);

    for (int i = 0; i < N - 1; i++) {
        for (int j = 0; j < N - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float tmp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }

    float median = (sorted[N / 2 - 1] + sorted[N / 2]) / 2.0;

    output[0] = mean;
    output[1] = std_val;
    output[2] = maxVal;
    output[3] = minVal;
    output[4] = maxVal - minVal;
    output[5] = median;
    output[6] = (float)energy;
}

// =========================
// RECEIVE CALLBACK
// =========================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {

    struct_message data;
    memcpy(&data, incomingData, sizeof(data));

    if (data.id == 1) {
        memcpy(pergelangan_features, data.features, sizeof(pergelangan_features));
        pergelanganReceived = true;
        Serial.println("✅ Data Pergelangan Tangan diterima!");
    }
    else if (data.id == 2) {
        memcpy(lenganAtas_features, data.features, sizeof(lenganAtas_features));
        lenganAtasReceived = true;
        Serial.println("✅ Data Lengan Atas diterima!");
    }
}

// =========================
// SETUP
// =========================
void setup() {
    Serial.begin(115200);
    Wire.begin(19, 18);
    pinMode(BUZZER_PIN, OUTPUT);

    Serial.println("Init MPU6050...");
    mpu.begin(0x68, &Wire);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.println("✅ MPU6050 Punggung OK!");

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init gagal!");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("✅ ESP-NOW READY!");
    Serial.println("Menunggu data Pergelangan Tangan & Lengan Atas...");
}

// =========================
// LOOP
// =========================
void loop() {

    // =========================
    // 1. TUNGGU PERGELANGAN TANGAN & LENGAN ATAS SELESAI KIRIM
    //    Trigger window punggung = begitu pergelangan dan lengan atas masuk.
    // =========================
    if (!pergelanganReceived || !lenganAtasReceived) {
        delay(50);
        return;
    }

    Serial.println("\n Pergelangan & Lengan Atas masuk, ambil window punggung...");

    // =========================
    // 2. AMBIL 40 SAMPEL PUNGGUNG (RAW → NORMALISASI)
    // =========================
    const int N = 40;
    float mag_arr[N];

    for (int i = 0; i < N; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float gx_n = normalize(g.gyro.x * GYRO_FACTOR, gx3_min, gx3_max);
        float gy_n = normalize(g.gyro.y * GYRO_FACTOR, gy3_min, gy3_max);
        float gz_n = normalize(g.gyro.z * GYRO_FACTOR, gz3_min, gz3_max);

        mag_arr[i] = sqrt(gx_n*gx_n + gy_n*gy_n + gz_n*gz_n);

        delay(50);
    }

    // =========================
    // 3. FEATURE EXTRACTION PUNGGUNG
    // =========================
    float punggung_features[7];
    extractFeature(mag_arr, N, punggung_features);

    Serial.println("\n===== FITUR PUNGGUNG =====");
    for (int i = 0; i < 7; i++) {
        Serial.print("F"); Serial.print(i);
        Serial.print(" : "); Serial.println(punggung_features[i]);
    }

    // =========================
    // 4. GABUNGKAN 21 FITUR
    // =========================
    float allFeatures[21];

    for (int i = 0; i < 7; i++) allFeatures[i]      = pergelangan_features[i];
    for (int i = 0; i < 7; i++) allFeatures[i + 7]  = lenganAtas_features[i];
    for (int i = 0; i < 7; i++) allFeatures[i + 14] = punggung_features[i];

    Serial.println("\n===== SEMUA FITUR =====");
    for (int i = 0; i < 21; i++) {
        Serial.print(allFeatures[i]);
        Serial.print(", ");
    }
    Serial.println();

    // =========================
    // 5. PREDICT
    // =========================

    int hasil = rf.predict(allFeatures);

    totalPercobaan++;

    Serial.print("\nRepetisi ke-");
    Serial.println(totalPercobaan);

    Serial.print("Hasil RF: ");
    Serial.println(hasil);

    if (hasil == 0) {
        totalBenar++;
        Serial.println("Hasil: BENAR ✅");

        // Buzzer 1x untuk BENAR
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
    }
    else {
        totalSalah++;
        Serial.println("Hasil: SALAH ❌");

        // Buzzer 3x untuk SALAH
        for (int i = 0; i < 3; i++) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(300);
            digitalWrite(BUZZER_PIN, LOW);
            delay(200);
        }
    }

    Serial.println("\n===== REKAP =====");
    Serial.print("Total Percobaan : "); Serial.println(totalPercobaan);
    Serial.print("Total Benar     : "); Serial.println(totalBenar);
    Serial.print("Total Salah     : "); Serial.println(totalSalah);

    float akurasi = (float)totalBenar / totalPercobaan * 100.0;
    Serial.print("Persentase Benar : ");
    Serial.print(akurasi);
    Serial.println("%");

    if (totalPercobaan >= TARGET_REPETISI) {
        Serial.println("\n========================");
        Serial.println("PENGUJIAN SELESAI");
        Serial.println("========================");
        Serial.print("Total Repetisi : "); Serial.println(totalPercobaan);
        Serial.print("Benar : "); Serial.println(totalBenar);
        Serial.print("Salah : "); Serial.println(totalSalah);

        float finalAcc = (float)totalBenar / totalPercobaan * 100.0;
        Serial.print("Akurasi : ");
        Serial.print(finalAcc);
        Serial.println("%");

        while (true) {
            delay(1000);
        }
    }

    // =========================
    // 6. RESET FLAG
    // =========================
    pergelanganReceived = false;
    lenganAtasReceived = false;

    Serial.println("\nMenunggu gerakan berikutnya...");
    delay(200);
}
