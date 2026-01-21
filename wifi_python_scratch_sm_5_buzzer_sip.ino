#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

/* ================= WIFI ================= */
const char* WIFI_SSID = "Ayush";
const char* WIFI_PASS = "6xcpic5kp9";

const char* SERVER_IP = "10.53.156.248";   // PC IP
const uint16_t SERVER_PORT = 8000;

WiFiClient client;

/* ================= PINS ================= */
#define IMU_ADDR 0x68
#define SDA_PIN 21
#define SCL_PIN 22
#define BUZZER_PIN 26

/* ================= VL53L0X ================= */
Adafruit_VL53L0X lox;

/* ================= IMU ================= */
int16_t ax_raw[3], ay_raw[3], az_raw[3];
float ax_f = 0, ay_f = 0, az_f = 0;
uint8_t idx = 0;

/* ================= BOTTLE ================= */
const float TILT_OK_THRESHOLD = 0.15;
const float BOTTLE_HEIGHT_CM  = 21.2;
const float BOTTLE_RADIUS_CM  = 3.85;
const float SENSOR_OFFSET_CM  = -8.3;

/* ================= TIMING ================= */
const uint32_t IMU_PERIOD_MS   = 50;
const uint32_t LIDAR_PERIOD_MS = 100;
const uint32_t NO_DRINK_MS     = 3600000UL;  // 60 min

uint32_t lastIMUTime   = 0;
uint32_t lastLidarTime = 0;

/* ================= DRINK LOGIC ================= */
const float DRINK_DELTA_ML = 15.0;

float reference_volume = -1;
uint32_t last_drink_time = 0;
bool buzzer_active = false;

/* ================= UTIL ================= */
int16_t read16(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 2);
  return (Wire.read() << 8) | Wire.read();
}

int16_t median3(int16_t a, int16_t b, int16_t c) {
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}

float ema(float prev, float curr, float alpha) {
  return alpha * curr + (1 - alpha) * prev;
}

void buzzerOn()  { digitalWrite(BUZZER_PIN, HIGH); }
void buzzerOff() { digitalWrite(BUZZER_PIN, LOW);  }

/* ================= WIFI CONNECT ================= */
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void connectServer() {
  while (!client.connect(SERVER_IP, SERVER_PORT)) {
    delay(1000);
  }
}

/* ================= SETUP ================= */
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  Wire.begin(SDA_PIN, SCL_PIN);

  connectWiFi();
  connectServer();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  if (!lox.begin()) {
    while (1);
  }

  last_drink_time = millis();
}

/* ================= LOOP ================= */
void loop() {

  if (!client.connected()) {
    connectServer();
  }

  /* ===== IMU ===== */
  if (millis() - lastIMUTime >= IMU_PERIOD_MS) {
    lastIMUTime = millis();

    ax_raw[idx] = read16(0x3B);
    ay_raw[idx] = read16(0x3D);
    az_raw[idx] = read16(0x3F);
    idx = (idx + 1) % 3;

    if (idx == 0) {
      int16_t ax = median3(ax_raw[0], ax_raw[1], ax_raw[2]);
      int16_t ay = median3(ay_raw[0], ay_raw[1], ay_raw[2]);
      int16_t az = median3(az_raw[0], az_raw[1], az_raw[2]);

      ax_f = ema(ax_f, ax, 0.15);
      ay_f = ema(ay_f, ay, 0.15);
      az_f = ema(az_f, az, 0.15);

      float g = sqrt(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
      if (g > 1e-3) {
        ax_f /= g; ay_f /= g; az_f /= g;
      }
    }
  }

  /* ===== LIDAR + DRINK LOGIC ===== */
  if (millis() - lastLidarTime >= LIDAR_PERIOD_MS) {
    lastLidarTime = millis();

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {

      float tilt_x = ax_f / az_f;
      float tilt_y = ay_f / az_f;
      bool orientation_ok =
        (sqrt(tilt_x*tilt_x + tilt_y*tilt_y) <= TILT_OK_THRESHOLD);

      uint32_t now = millis();
      bool sip_detected = false;

      float volume_ml = 0;
      float percent_full = 0;

      if (orientation_ok) {
        float distance_cm = measure.RangeMilliMeter / 10.0;
        float water_height_cm =
          BOTTLE_HEIGHT_CM - distance_cm - SENSOR_OFFSET_CM;

        water_height_cm = constrain(water_height_cm, 0, BOTTLE_HEIGHT_CM);

        volume_ml =
          3.14159 * BOTTLE_RADIUS_CM * BOTTLE_RADIUS_CM * water_height_cm;

        percent_full =
          (water_height_cm / BOTTLE_HEIGHT_CM) * 100.0;

        if (reference_volume < 0) {
          reference_volume = volume_ml;
          last_drink_time = now;
        }

        if (reference_volume - volume_ml > DRINK_DELTA_ML) {
          sip_detected = true;
          reference_volume = volume_ml;
          last_drink_time = now;
          buzzerOff();
          buzzer_active = false;
        }
      }

      if (!buzzer_active && now - last_drink_time >= NO_DRINK_MS) {
        buzzerOn();
        buzzer_active = true;
      }

      uint32_t sec_since = (now - last_drink_time) / 1000;
      uint32_t sec_left =
        (now - last_drink_time >= NO_DRINK_MS) ? 0 :
        (NO_DRINK_MS - (now - last_drink_time)) / 1000;

      /* ===== SEND CSV OVER WIFI ===== */
      client.print(volume_ml, 0);
      client.print(",");
      client.print(percent_full, 1);
      client.print(",");
      client.print(sec_since);
      client.print(",");
      client.print(sec_left);
      client.print(",");
      client.print(orientation_ok ? 1 : 0);
      client.print(",");
      client.println(sip_detected ? 1 : 0);
    }
  }
}
