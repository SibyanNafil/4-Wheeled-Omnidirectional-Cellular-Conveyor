// ===== SLAVE I2C - ESP32 =====
// Menerima 8 byte dari master: [dir1,pwm1, dir2,pwm2, dir3,pwm3, dir4,pwm4]
// Penentuan dir dari arah pandangan luar cell
// dir = 1 -> CW, dir = 0 -> CCW
//
// PIN_PROX = sensor proximity (input). Nilainya dikirim ke master saat
// master melakukan Wire.requestFrom(SLAVE_ID, 1).
//
// LED L_MAJU/L_KANAN/L_MUNDUR/L_KIRI = Indikator Arah gerakan Omni
// Kombinasi arah motor untuk tiap indikator (lihat updateLedArah()):
//   MAJU   : M1=CCW(b), M2=CW(a),  M3=CW(a),  M4=CCW(b)
//   KANAN  : M1=CCW(b), M2=CCW(b), M3=CW(a),  M4=CW(b)
//   MUNDUR : M1=CW(a),  M2=CCW(b), M3=CCW(b), M4=CW(a)
//   KIRI   : M1=CW(a),  M2=CW(a),  M3=CCW(b), M4=CCW(b)

#include <Arduino.h>
#include <Wire.h>
#define SLAVE_ID 2
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000
#define JUMLAH_MOTOR 4

static constexpr uint8_t PIN_PROX = 23;

const uint8_t PIN_M1_A1 = 13;
const uint8_t PIN_M1_A2 = 12;
const uint8_t PIN_M2_A3 = 14;
const uint8_t PIN_M2_A4 = 27;
const uint8_t PIN_M3_A1 = 33;
const uint8_t PIN_M3_A2 = 32;
const uint8_t PIN_M4_A3 = 26;
const uint8_t PIN_M4_A4 = 25;

// // selebumnya
// const uint8_t PIN_M1_A1 = 14; const uint8_t PIN_M1_A2 = 27;
// const uint8_t PIN_M2_A3 = 13; const uint8_t PIN_M2_A4 = 12;
// const uint8_t PIN_M3_A1 = 26; const uint8_t PIN_M3_A2 = 25;
// const uint8_t PIN_M4_A3 = 33; const uint8_t PIN_M4_A4 = 32;

const uint8_t L_MAJU = 19;
const uint8_t L_KANAN = 18;
const uint8_t L_MUNDUR = 5;
const uint8_t L_KIRI = 15;

// Kelompokkan pin per motor: {pinA, pinB}
const uint8_t motorPinA[JUMLAH_MOTOR] = {PIN_M1_A1, PIN_M2_A3, PIN_M3_A1, PIN_M4_A3};
const uint8_t motorPinB[JUMLAH_MOTOR] = {PIN_M1_A2, PIN_M2_A4, PIN_M3_A2, PIN_M4_A4};

// Channel LEDC (core lama arduino-esp32 < 3.0). 2 channel per motor -> 8 channel total.
const int pwmFreq = 5000;
const int pwmResolution = 8; // 0-255

volatile uint8_t motorDir[JUMLAH_MOTOR] = {0, 0, 0, 0}; // 1=CW, 0=CCW
volatile uint8_t motorPwm[JUMLAH_MOTOR] = {0, 0, 0, 0};
volatile bool dataBaru = false;
volatile bool proxState = false;

void onI2CReceive(int len)
{
    uint8_t buf[JUMLAH_MOTOR * 2];
    int i = 0;
    while (Wire.available() && i < JUMLAH_MOTOR * 2)
    {
        buf[i++] = Wire.read();
    }
    if (i == JUMLAH_MOTOR * 2)
    {
        for (int m = 0; m < JUMLAH_MOTOR; m++)
        {
            motorDir[m] = buf[m * 2];
            motorPwm[m] = buf[m * 2 + 1];
        }
        dataBaru = true;
    }
}

void onI2CRequest()
{
    Wire.write((uint8_t)(proxState ? 1 : 0));
}

void terapkanMotor(int idx)
{
    int chA = idx * 2;
    int chB = idx * 2 + 1;

    if (motorDir[idx] == 1)
    {
        // CW -> PWM di pin A, pin B LOW
        ledcWrite(chA, motorPwm[idx]);
        ledcWrite(chB, 0);
    }
    else
    {
        // CCW -> PWM di pin B, pin A LOW
        ledcWrite(chA, 0);
        ledcWrite(chB, motorPwm[idx]);
    }

    // --- Kalau pakai core arduino-esp32 >= 3.0, ganti 6 baris di atas dengan: ---
    // if (motorDir[idx] == 1) {
    //   ledcWrite(motorPinA[idx], motorPwm[idx]);
    //   ledcWrite(motorPinB[idx], 0);
    // } else {
    //   ledcWrite(motorPinA[idx], 0);
    //   ledcWrite(motorPinB[idx], motorPwm[idx]);
    // }
}

// Indikator LED arah gerak robot berdasarkan kombinasi arah motor.
// Kombinasi arah (1 = CW, 0 = CCW) untuk tiap indikator:
//   MAJU   : M1=CCW, M2=CW,  M3=CW,  M4=CCW
//   KANAN  : M1=CCW, M2=CCW, M3=CW,  M4=CW
//   MUNDUR : M1=CW,  M2=CCW, M3=CCW, M4=CW
//   KIRI   : M1=CW,  M2=CW,  M3=CCW, M4=CCW
// Semua motor juga harus punya PWM > 150 supaya dianggap benar-benar bergerak (cukup kencang).
static constexpr uint8_t PWM_AMBANG_LED = 150;

// LED yang dipakai aktif-LOW: sinyal LOW = LED menyala, HIGH = LED mati.
static inline void ledTulis(uint8_t pin, bool nyala)
{
    digitalWrite(pin, nyala ? LOW : HIGH);
}

void updateLedArah()
{
    bool semuaJalan = (motorPwm[0] > PWM_AMBANG_LED) && (motorPwm[1] > PWM_AMBANG_LED) &&
                      (motorPwm[2] > PWM_AMBANG_LED) && (motorPwm[3] > PWM_AMBANG_LED);

    bool maju = semuaJalan &&
                (motorDir[0] == 0) && (motorDir[1] == 1) &&
                (motorDir[2] == 1) && (motorDir[3] == 0);

    bool kanan = semuaJalan &&
                 (motorDir[0] == 0) && (motorDir[1] == 0) &&
                 (motorDir[2] == 1) && (motorDir[3] == 1);

    bool mundur = semuaJalan &&
                  (motorDir[0] == 1) && (motorDir[1] == 0) &&
                  (motorDir[2] == 0) && (motorDir[3] == 1);

    bool kiri = semuaJalan &&
                (motorDir[0] == 1) && (motorDir[1] == 1) &&
                (motorDir[2] == 0) && (motorDir[3] == 0);

    ledTulis(L_MAJU, maju);
    ledTulis(L_MUNDUR, mundur);
    ledTulis(L_KANAN, kanan);
    ledTulis(L_KIRI, kiri);
}

void setup()
{
    Serial.begin(115200);
    delay(50);

    pinMode(PIN_PROX, INPUT);
    pinMode(L_MAJU, OUTPUT);
    pinMode(L_KANAN, OUTPUT);
    pinMode(L_MUNDUR, OUTPUT);
    pinMode(L_KIRI, OUTPUT);
    // LED aktif-LOW -> HIGH = mati, jadi set HIGH dulu saat start
    digitalWrite(L_MAJU, HIGH);
    digitalWrite(L_KANAN, HIGH);
    digitalWrite(L_MUNDUR, HIGH);
    digitalWrite(L_KIRI, HIGH);

    for (int i = 0; i < JUMLAH_MOTOR; i++)
    {
        pinMode(motorPinA[i], OUTPUT);
        pinMode(motorPinB[i], OUTPUT);
        digitalWrite(motorPinA[i], LOW);
        digitalWrite(motorPinB[i], LOW);

        // --- Setup LEDC (core lama arduino-esp32 < 3.0) ---
        ledcSetup(i * 2, pwmFreq, pwmResolution);
        ledcSetup(i * 2 + 1, pwmFreq, pwmResolution);
        ledcAttachPin(motorPinA[i], i * 2);
        ledcAttachPin(motorPinB[i], i * 2 + 1);

        // --- Kalau core >= 3.0, ganti 4 baris di atas dengan: ---
        // ledcAttach(motorPinA[i], pwmFreq, pwmResolution);
        // ledcAttach(motorPinB[i], pwmFreq, pwmResolution);
    }

    Wire.begin((uint8_t)SLAVE_ID, I2C_SDA, I2C_SCL, I2C_FREQ);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    Serial.print("Slave I2C siap, alamat: ");
    Serial.println(SLAVE_ID);
}

void loop()
{
    // Baca sensor proximity terus-menerus
    proxState = digitalRead(PIN_PROX);

    if (dataBaru)
    {
        dataBaru = false;

        for (int i = 0; i < JUMLAH_MOTOR; i++)
        {
            terapkanMotor(i);
        }
        updateLedArah();

        Serial.print("Update motor -> ");
        for (int i = 0; i < JUMLAH_MOTOR; i++)
        {
            Serial.print("M");
            Serial.print(i + 1);
            Serial.print(":");
            Serial.print(motorDir[i] ? "CW" : "CCW");
            Serial.print("/");
            Serial.print(motorPwm[i]);
            Serial.print("  ");
        }
        Serial.print("| PROX: ");
        Serial.println(proxState ? "1" : "0");
    }
}
