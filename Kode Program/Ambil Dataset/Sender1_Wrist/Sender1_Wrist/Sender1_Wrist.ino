#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

// =========================
// MAC ADDRESS ESP32 DADA
// =========================
uint8_t dadaAddress[] = {0x60, 0x55, 0xF9, 0x71, 0x62, 0xB0};

// =========================
// MPU
// =========================
Adafruit_MPU6050 mpu;

// =========================
// STRUCT DATA
// =========================
typedef struct struct_message {

    int id;
    float features[7];

} struct_message;

struct_message dataWrist;

// =========================
// CALLBACK ESP-NOW
// =========================
void OnDataSent(const wifi_tx_info_t *info,
                esp_now_send_status_t status) {

    Serial.print("Send Status: ");

    if (status == ESP_NOW_SEND_SUCCESS)
        Serial.println("Success");

    else
        Serial.println("Fail");
}

// =========================
// SETUP
// =========================
void setup() {

    Serial.begin(115200);

    // =========================
    // I2C
    // =========================
    Wire.begin(19, 18);

    Serial.println("Init MPU6050...");

    // =========================
    // MPU6050
    // =========================
    mpu.begin(0x68, &Wire);

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    Serial.println("MPU6050 Wrist OK!");

    // =========================
    // WIFI
    // =========================
    WiFi.mode(WIFI_STA);

    // =========================
    // ESP-NOW
    // =========================
    if (esp_now_init() != ESP_OK) {

        Serial.println("ESP-NOW init gagal!");

        return;
    }

    Serial.println("ESP-NOW READY!");

    esp_now_register_send_cb(OnDataSent);

    // =========================
    // PEER
    // =========================
    esp_now_peer_info_t peerInfo;

    memset(&peerInfo, 0, sizeof(peerInfo));

    memcpy(peerInfo.peer_addr, dadaAddress, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {

        Serial.println("Gagal tambah peer!");

        return;
    }

    Serial.println("Peer berhasil ditambah!");
}

// =========================
// LOOP
// =========================
void loop() {

    const int N = 40;

    float mag_arr[N];

    // =========================
    // AMBIL DATA GYRO
    // =========================
    for (int i = 0; i < N; i++) {

        sensors_event_t a, g, temp;

        mpu.getEvent(&a, &g, &temp);

        mag_arr[i] =
            sqrt(
                g.gyro.x * g.gyro.x +
                g.gyro.y * g.gyro.y +
                g.gyro.z * g.gyro.z
            );

        delay(50);
    }

    // =========================
    // FEATURE EXTRACTION
    // =========================
    float sum = 0;
    float energy = 0;

    float maxVal = mag_arr[0];
    float minVal = mag_arr[0];

    for (int i = 0; i < N; i++) {

        sum += mag_arr[i];

        energy += mag_arr[i] * mag_arr[i];

        if (mag_arr[i] > maxVal)
            maxVal = mag_arr[i];

        if (mag_arr[i] < minVal)
            minVal = mag_arr[i];
    }

    float mean = sum / N;

    // =========================
    // STD
    // =========================
    float std_val = 0;

    for (int i = 0; i < N; i++) {

        std_val +=
            (mag_arr[i] - mean) *
            (mag_arr[i] - mean);
    }

    std_val = sqrt(std_val / N);

    // =========================
    // MEDIAN
    // =========================
    float sorted[N];

    memcpy(sorted, mag_arr, sizeof(mag_arr));

    for (int i = 0; i < N - 1; i++) {

        for (int j = 0; j < N - i - 1; j++) {

            if (sorted[j] > sorted[j + 1]) {

                float tmp = sorted[j];

                sorted[j] = sorted[j + 1];

                sorted[j + 1] = tmp;
            }
        }
    }

    float median =
        (sorted[N / 2 - 1] + sorted[N / 2]) / 2.0;

    // =========================
    // SIMPAN FEATURE
    // =========================
    dataWrist.id = 1;

    dataWrist.features[0] = mean;
    dataWrist.features[1] = std_val;
    dataWrist.features[2] = maxVal;
    dataWrist.features[3] = minVal;
    dataWrist.features[4] = maxVal - minVal;
    dataWrist.features[5] = median;
    dataWrist.features[6] = energy;

    // =========================
    // DEBUG FEATURE
    // =========================
    Serial.println("=== Wrist Features ===");

    for (int i = 0; i < 7; i++) {

        Serial.print("F");
        Serial.print(i);
        Serial.print(": ");

        Serial.println(dataWrist.features[i]);
    }

    // =========================
    // KIRIM ESPNOW
    // =========================
    esp_now_send(
        dadaAddress,
        (uint8_t *) &dataWrist,
        sizeof(dataWrist)
    );

    Serial.println("Data Wrist terkirim!");
    Serial.println();

    delay(500);
}