
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <string.h>

// ==================== CAU HINH PHAN CUNG ====================
constexpr uint8_t PIN_TRIG  = 5;
constexpr uint8_t PIN_ECHO  = 18;
constexpr uint8_t PIN_TEMP  = 4;
constexpr uint8_t PIN_PUMP  = 25;
constexpr uint8_t PIN_VALVE = 26;

// ====================== WIFI / MQTT =========================
const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASS = "";
const char *MQTT_HOST = "broker.emqx.io";
constexpr uint16_t MQTT_PORT = 1883;

const char *TOPIC_TELEMETRY = "aquafarm/wokwi-dang1806/telemetry";
const char *TOPIC_COMMAND   = "aquafarm/wokwi-dang1806/command";
const char *TOPIC_STATE     = "aquafarm/wokwi-dang1806/state";
const char *TOPIC_ONLINE    = "aquafarm/wokwi-dang1806/online";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
OneWire oneWire(PIN_TEMP);
DallasTemperature ds18b20(&oneWire);

struct SensorData {
  float distanceM;        // Khoang cach dau do -> mat nuoc (met)
  float temperatureC;     // Nhiet do nuoc (do C)
  uint32_t timeMs;
};

QueueHandle_t dataQueue  = nullptr;   // Day du ca hai thong so
QueueHandle_t dataAlert  = nullptr;   // Chi khoang cach

enum ControlMode : uint8_t { MODE_AUTO = 0, MODE_MANUAL = 1 };
enum WaterZone : uint8_t {
  Z_UNKNOWN = 0,
  Z_IDLE,
  Z_DRAIN,
  Z_PUMP50,
  Z_PUMP100
};

// Bao ve bien dung chung giua 2 core (critical section that ngan).
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;
ControlMode controlMode = MODE_AUTO;
bool manualPump = false;
bool manualValve = false;
uint8_t pumpDuty = 0;   // 0/128/255
bool valveOpen = false;

// ====================== HAM TIEN ICH ========================
float measureDistanceM() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long pulseUs = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (pulseUs == 0) return NAN;
  float centimeters = pulseUs / 58.0f;
  if (centimeters < 2.0f || centimeters > 400.0f) return NAN;
  return centimeters / 100.0f;
}

WaterZone getWaterZone(float distanceM) {
  if (!isfinite(distanceM)) return Z_UNKNOWN;
  if (distanceM < 0.5f) return Z_DRAIN;
  if (distanceM >= 1.0f && distanceM < 1.5f) return Z_PUMP50;
  if (distanceM >= 1.5f && distanceM <= 2.0f) return Z_PUMP100;
  return Z_IDLE;  // Khoang 0.5..1.0m va tren 2.0m: khong tac dong (de khong neu).
}

void applyOutputs(uint8_t duty, bool valve) {
  // Khoa lien dong: khong cho bom va xa cung luc.
  if (valve) duty = 0;
  analogWrite(PIN_PUMP, duty);   // ESP32 PWM: 0..255; 128 xap xi 50%
  digitalWrite(PIN_VALVE, valve ? HIGH : LOW);
  portENTER_CRITICAL(&controlMux);
  pumpDuty = duty;
  valveOpen = valve;
  portEXIT_CRITICAL(&controlMux);
}

// MQTT callback chi thay doi LENH, TaskCanhBao (Core 1) moi dieu khien GPIO.
void onMqttMessage(char *topic, byte *payload, unsigned int len) {
  if (strcmp(topic, TOPIC_COMMAND) != 0 || len == 0 || len >= 40) return;
  char cmd[40];
  memcpy(cmd, payload, len);
  cmd[len] = '\0';

  portENTER_CRITICAL(&controlMux);
  if (strcmp(cmd, "MODE:AUTO") == 0) {
    controlMode = MODE_AUTO;
    manualPump = false;
    manualValve = false;
  } else if (strcmp(cmd, "MODE:MANUAL") == 0) {
    controlMode = MODE_MANUAL;
    manualPump = false;
    manualValve = false;
  } else if (controlMode == MODE_MANUAL) {
    if (strcmp(cmd, "PUMP:ON") == 0) {
      manualPump = true;
      manualValve = false;
    } else if (strcmp(cmd, "PUMP:OFF") == 0) {
      manualPump = false;
    } else if (strcmp(cmd, "VALVE:ON") == 0) {
      manualValve = true;
      manualPump = false;
    } else if (strcmp(cmd, "VALVE:OFF") == 0) {
      manualValve = false;
    }
  }
  portEXIT_CRITICAL(&controlMux);
  Serial.printf("[MQTT] Lenh: %s\n", cmd);
}

void publishTelemetry(const SensorData &d) {
  if (!mqtt.connected()) return;
  char dist[20], temp[20], payload[160];
  if (isfinite(d.distanceM)) snprintf(dist, sizeof(dist), "%.3f", d.distanceM);
  else strcpy(dist, "null");
  if (isfinite(d.temperatureC)) snprintf(temp, sizeof(temp), "%.2f", d.temperatureC);
  else strcpy(temp, "null");
  snprintf(payload, sizeof(payload),
           "{\"distanceM\":%s,\"temperatureC\":%s,\"timeMs\":%lu}",
           dist, temp, (unsigned long)d.timeMs);
  mqtt.publish(TOPIC_TELEMETRY, payload); // Du lieu mau khong retained.
}

void publishState() {
  if (!mqtt.connected()) return;
  ControlMode mode;
  uint8_t duty;
  bool valve;
  portENTER_CRITICAL(&controlMux);
  mode = controlMode;
  duty = pumpDuty;
  valve = valveOpen;
  portEXIT_CRITICAL(&controlMux);
  char payload[112];
  snprintf(payload, sizeof(payload),
           "{\"mode\":\"%s\",\"pumpPwm\":%u,\"valve\":%s}",
           mode == MODE_AUTO ? "AUTO" : "MANUAL",
           (unsigned int)duty, valve ? "true" : "false");
  mqtt.publish(TOPIC_STATE, payload, true); // Retained: web moi vao co trang thai.
}

// ===================== CORE 0 ==============================
void TaskTruyenData(void *pv) {
  WiFi.mode(WIFI_STA);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(256);
  mqtt.setKeepAlive(30);

  uint32_t lastWifiTry = 0;
  uint32_t lastMqttTry = 0;
  uint32_t lastStatePub = 0;

  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (lastWifiTry == 0 || (uint32_t)(now - lastWifiTry) >= 5000UL) {
        lastWifiTry = now;
        Serial.println("[Core0] Dang ket noi Wi-Fi...");
        WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!mqtt.connected()) {
      if (lastMqttTry == 0 || (uint32_t)(now - lastMqttTry) >= 3000UL) {
        lastMqttTry = now;
        char clientId[48];
        snprintf(clientId, sizeof(clientId), "wokwi-aqua-%04X",
                 (unsigned int)(ESP.getEfuseMac() & 0xFFFF));
        Serial.println("[Core0] Dang ket noi MQTT...");
        if (mqtt.connect(clientId, TOPIC_ONLINE, 0, true, "offline")) {
          mqtt.subscribe(TOPIC_COMMAND);
          mqtt.publish(TOPIC_ONLINE, "online", true);
          publishState();
          Serial.println("[Core0] MQTT da ket noi, da subscribe command.");
        } else {
          Serial.printf("[Core0] Loi MQTT, rc=%d\n", mqtt.state());
        }
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    mqtt.loop();
    SensorData d;
    if (xQueueReceive(dataQueue, &d, pdMS_TO_TICKS(20)) == pdTRUE) {
      publishTelemetry(d);
    }
    now = millis();
    if ((uint32_t)(now - lastStatePub) >= 500UL) {
      lastStatePub = now;
      publishState();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===================== CORE 1 ==============================
void TaskDocData(void *pv) {
  ds18b20.begin();
  ds18b20.setResolution(10); // Thoi gian chuyen doi ~188ms, phu hop chu ky 2s.

  for (;;) {
    SensorData d;
    d.distanceM = measureDistanceM();
    ds18b20.requestTemperatures();
    d.temperatureC = ds18b20.getTempCByIndex(0);
    if (!isfinite(d.temperatureC) || d.temperatureC == DEVICE_DISCONNECTED_C ||
        d.temperatureC < -55.0f || d.temperatureC > 125.0f) {
      d.temperatureC = NAN;
    }
    d.timeMs = millis();

    Serial.printf("[Core1][DOC] Khoang cach: %.3f m | Nhiet do: %.2f C\n",
                  d.distanceM, d.temperatureC);

    // Queue du lieu day du: neu day, bo mau cu de uu tien mau moi.
    if (xQueueSend(dataQueue, &d, 0) != pdTRUE) {
      SensorData dropped;
      xQueueReceive(dataQueue, &dropped, 0);
      xQueueSend(dataQueue, &d, 0);
    }
    // Queue canh bao chi co 1 phan tu, luon ghi de bang gia tri moi.
    xQueueOverwrite(dataAlert, &d.distanceM);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskCanhBao(void *pv) {
  ControlMode previousMode = MODE_MANUAL; // Ep kiem tra tu dau o che do AUTO.
  WaterZone lastZone = Z_UNKNOWN;
  uint8_t autoDuty = 0;
  bool autoValve = false;
  uint32_t autoEndMs = 0;

  applyOutputs(0, false);
  for (;;) {
    ControlMode mode;
    bool mPump, mValve;
    portENTER_CRITICAL(&controlMux);
    mode = controlMode;
    mPump = manualPump;
    mValve = manualValve;
    portEXIT_CRITICAL(&controlMux);

    if (mode != previousMode) {
      previousMode = mode;
      lastZone = Z_UNKNOWN;
      autoDuty = 0;
      autoValve = false;
      autoEndMs = 0;
      applyOutputs(0, false);
      Serial.printf("[Core1][ALERT] Chuyen che do: %s\n",
                    mode == MODE_AUTO ? "AUTO" : "MANUAL");
    }

    float distanceM;
    BaseType_t gotData = xQueueReceive(dataAlert, &distanceM,
                                       pdMS_TO_TICKS(100));
    if (mode == MODE_MANUAL) {
      applyOutputs(mPump ? 255 : 0, mValve);
      continue;
    }

    if (gotData == pdTRUE) {
      WaterZone currentZone = getWaterZone(distanceM);
      // Chi kich hoat khi CHUYEN VUNG: tranh bat lai moi 2s vo han.
      if (currentZone != lastZone) {
        lastZone = currentZone;
        autoDuty = 0;
        autoValve = false;
        uint32_t durationMs = 0;
        switch (currentZone) {
          case Z_DRAIN:   autoValve = true; durationMs = 10000UL; break;
          case Z_PUMP50:  autoDuty = 128;    durationMs = 10000UL; break;
          case Z_PUMP100: autoDuty = 255;    durationMs = 20000UL; break;
          default: break;
        }
        autoEndMs = millis() + durationMs;
        Serial.printf("[Core1][ALERT] Vung=%d, bom=%u, van=%d, thoi gian=%lu ms\n",
                      (int)currentZone, (unsigned)autoDuty, (int)autoValve,
                      (unsigned long)durationMs);
      }
    }

    // Kiem tra het thoi gian, khong block task trong 10/20 giay.
    if ((autoDuty != 0 || autoValve) &&
        (int32_t)(millis() - autoEndMs) >= 0) {
      autoDuty = 0;
      autoValve = false;
      Serial.println("[Core1][ALERT] Da het thoi gian: TAT co cau.");
    }
    applyOutputs(autoDuty, autoValve);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_VALVE, LOW);

  dataQueue = xQueueCreate(8, sizeof(SensorData));
  dataAlert = xQueueCreate(1, sizeof(float));
  if (!dataQueue || !dataAlert) {
    Serial.println("KHONG TAO DUOC QUEUE!");
    while (true) delay(1000);
  }

  xTaskCreatePinnedToCore(TaskTruyenData, "TaskTruyenData", 8192,
                          nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(TaskDocData, "TaskDocData", 4096,
                          nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCanhBao, "TaskCanhBao", 4096,
                          nullptr, 3, nullptr, 1);
  Serial.println("[SYSTEM] FreeRTOS: Truyen Core0; Doc+CanhBao Core1");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
