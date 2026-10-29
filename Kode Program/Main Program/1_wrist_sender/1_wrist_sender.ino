#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

#define GYRO_FACTOR 3752.9

// MAC ADDRESS ESP32 PUNGGUNG
uint8_t punggungAddress[] = {0x60, 0x55, 0xF9, 0x71, 0x62, 0xB0};

// MPU
Adafruit_MPU6050 mpu;

// Min-max BARU
float gx1_min=-31176, gx1_max=27564;
float gy1_min=-26507, gy1_max=17205;
float gz1_min=-32707, gz1_max=32765;

float normalize(float val, float minVal, float maxVal) {
    return (val - minVal) / (maxVal - minVal);
}

// STRUCT DATA
typedef struct struct_message {

    int id;
    float features[7];

} struct_message;

struct_message dataPergelangan;

// CALLBACK ESP-NOW
void OnDataSent(const wifi_tx_info_t *info,
                esp_now_send_status_t status) {

    Serial.print("Send Status: ");

    if (status == ESP_NOW_SEND_SUCCESS)
        Serial.println("Success \n");

    else
        Serial.println("Fail \n");
}


// SETUP
void setup() {

    Serial.begin(115200);

    // I2C
    Wire.begin(19, 18);

    Serial.println("Init MPU6050...");

    // MPU6050
    mpu.begin(0x68, &Wire);

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    Serial.println("MPU6050 Pergelangan Tangan OK!");

    // WIFI
    WiFi.mode(WIFI_STA);

    // ESP-NOW
    if (esp_now_init() != ESP_OK) {

        Serial.println("ESP-NOW init gagal!");

        return;
    }

    Serial.println("ESP-NOW READY!");

    esp_now_register_send_cb(OnDataSent);

    // PEER
    esp_now_peer_info_t peerInfo;

    memset(&peerInfo, 0, sizeof(peerInfo));

    memcpy(peerInfo.peer_addr, punggungAddress, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {

        Serial.println("Gagal tambah peer!");

        return;
    }

    Serial.println("Peer berhasil ditambah!");
}


// LOOP
void loop() {

// TUNGGU GERAKAN
const float THRESHOLD = 0.8;

Serial.println("Menunggu gerakan...\n");

while (true) {

    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    float mag = sqrt(
        g.gyro.x * g.gyro.x +
        g.gyro.y * g.gyro.y +
        g.gyro.z * g.gyro.z
    );

    if (mag > THRESHOLD) {

        Serial.println("Gerakan terdeteksi!");

        delay(150);

        break;
    }

    delay(50);
}

    const int N = 40;

    float mag_arr[N];

    // AMBIL DATA GYRO
    for (int i = 0; i < N; i++) {

        sensors_event_t a, g, temp;

        mpu.getEvent(&a, &g, &temp);

        float gx_n = 
            normalize(
                g.gyro.x * GYRO_FACTOR,
                gx1_min,
                gx1_max
            );

        float gy_n =
            normalize(
                g.gyro.y * GYRO_FACTOR,
                gy1_min,
                gy1_max
            );

        float gz_n =
            normalize(
                g.gyro.z * GYRO_FACTOR,
                gz1_min,
                gz1_max
            );

        mag_arr[i] =
            sqrt(
                gx_n * gx_n +
                gy_n * gy_n +
                gz_n * gz_n
            );

        delay(50);
    }

    // FEATURE EXTRACTION
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

    // STD
    float std_val = 0;

    for (int i = 0; i < N; i++) {

        std_val +=
            (mag_arr[i] - mean) *
            (mag_arr[i] - mean);
    }

    std_val = sqrt(std_val / N);

    // MEDIAN
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

    // SIMPAN FEATURE
    dataPergelangan.id = 1;

    dataPergelangan.features[0] = mean;
    dataPergelangan.features[1] = std_val;
    dataPergelangan.features[2] = maxVal;
    dataPergelangan.features[3] = minVal;
    dataPergelangan.features[4] = maxVal - minVal;
    dataPergelangan.features[5] = median;
    dataPergelangan.features[6] = energy;

    // DEBUG FEATURE
    Serial.println("=== Fitur Pergelangan Tangan ===");

    for (int i = 0; i < 7; i++) {

        Serial.print("F");
        Serial.print(i);
        Serial.print(": ");

        Serial.println(dataPergelangan.features[i]);
    }

    // KIRIM ESPNOW
    esp_now_send(
        punggungAddress,
        (uint8_t *) &dataPergelangan,
        sizeof(dataPergelangan)
    );

    Serial.println("Data Pergelangan Tangan terkirim!");
    Serial.println();

    delay(500);
}

