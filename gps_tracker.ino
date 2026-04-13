#include <PubSubClient.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <DHT.h>
#include <LittleFS.h>

#define TRIGGER_PIN 0
#define DHT_PIN     21
#define DHT_TYPE    DHT11

#define LED_GPS  22
#define LED_WIFI 23
#define LED_MQTT 13

#define RX_PIN 18
#define TX_PIN 19

#define RAM_BUFFER_SIZE        200
#define MAX_QUEUE_SIZE        5000
#define BATCH_SIZE              15
#define TELEMETRY_INTERVAL    5000
#define MAX_STORAGE_KB         512
#define QUEUE_FILE        "/queue.dat"
#define METADATA_SAVE_INTERVAL 100
#define JSON_BUFFER_SIZE      5120

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
WiFiManager wm;

char mqtt_server[61]  = "";
char mqtt_port[11]    = "";
char device_id[33]    = "";
char device_token[65] = "";

bool shouldSaveConfig = false;
bool dhtWorking       = false;

struct TelemetryData {
  float    latitude;
  float    longitude;
  float    speed;
  float    bearing;
  float    temperature;
  float    humidity;
  char     datetime[25];
  uint8_t  flags;
} __attribute__((packed));

#define FLAG_HAS_SPEED    0x01
#define FLAG_HAS_BEARING  0x02
#define FLAG_HAS_TEMP     0x04

TelemetryData ramBuffer[RAM_BUFFER_SIZE];
int ramHead = 0;
int ramTail = 0;
int ramSize = 0;

int queueHead = 0;
int queueTail = 0;
int queueSize = 0;
int flashWritesSinceLastSave = 0;

unsigned long lastGpsValid  = 0;
unsigned long ledBlinkTimer = 0;
bool          ledBlinkState = false;

void updateLEDs() {
  unsigned long now = millis();

  if (gps.location.isValid()) {
    digitalWrite(LED_GPS, HIGH);
    lastGpsValid = now;
  } else {
    if (now - ledBlinkTimer > 500) {
      ledBlinkState = !ledBlinkState;
      ledBlinkTimer = now;
    }
    digitalWrite(LED_GPS, ledBlinkState ? HIGH : LOW);
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_WIFI, HIGH);
  } else {
    if (now - ledBlinkTimer > 200) {
      digitalWrite(LED_WIFI, ledBlinkState ? HIGH : LOW);
    }
  }

  int totalQueued = ramSize + queueSize;
  if (client.connected()) {
    if (totalQueued > 0) {
      if (now - ledBlinkTimer > 100) {
        digitalWrite(LED_MQTT, ledBlinkState ? HIGH : LOW);
      }
    } else {
      digitalWrite(LED_MQTT, HIGH);
    }
  } else {
    digitalWrite(LED_MQTT, LOW);
  }
}

void saveQueueMetadata() {
  preferences.begin("queue", false);
  preferences.putInt("head", queueHead);
  preferences.putInt("tail", queueTail);
  preferences.putInt("size", queueSize);
  preferences.end();
  flashWritesSinceLastSave = 0;
}

void loadQueueMetadata() {
  preferences.begin("queue", true);
  queueHead = preferences.getInt("head", 0);
  queueTail = preferences.getInt("tail", 0);
  queueSize = preferences.getInt("size", 0);
  preferences.end();

  if (queueSize < 0 || queueSize > MAX_QUEUE_SIZE) {
    Serial.println("Invalid queue metadata, resetting");
    queueHead = queueTail = queueSize = 0;
    saveQueueMetadata();
  }

  Serial.printf("Flash queue: %d items (head=%d, tail=%d)\n",
                queueSize, queueHead, queueTail);
}

void initFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS Mounted");

  size_t total = LittleFS.totalBytes();
  size_t used  = LittleFS.usedBytes();
  Serial.printf("Total: %d KB | Used: %d KB | Free: %d KB\n",
                total/1024, used/1024, (total-used)/1024);

  loadQueueMetadata();
}

bool checkStorageSpace() {
  size_t used  = LittleFS.usedBytes();
  size_t total = LittleFS.totalBytes();
  if ((total - used) < 10240)               return false;
  if (used > (size_t)(MAX_STORAGE_KB*1024)) return false;
  return true;
}

void clearFlashQueue() {
  queueHead = queueTail = queueSize = 0;
  saveQueueMetadata();
  LittleFS.remove(QUEUE_FILE);
  Serial.println("Flash queue cleared");
}

bool enqueueRam(const TelemetryData& data) {
  if (ramSize >= RAM_BUFFER_SIZE) return false;
  ramBuffer[ramTail] = data;
  ramTail = (ramTail + 1) % RAM_BUFFER_SIZE;
  ramSize++;
  return true;
}

bool dequeueRam(TelemetryData& data) {
  if (ramSize == 0) return false;
  data    = ramBuffer[ramHead];
  ramHead = (ramHead + 1) % RAM_BUFFER_SIZE;
  ramSize--;
  return true;
}

bool enqueueFlashBatch(const TelemetryData* items, int count) {
  if (count <= 0) return false;

  if (!checkStorageSpace()) {
    for (int i = 0; i < 10 && queueSize > 0; i++) {
      queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
      queueSize--;
    }
  }

  File file = LittleFS.open(QUEUE_FILE, "r+");
  if (!file) {
    file = LittleFS.open(QUEUE_FILE, "w+");
    if (!file) {
      Serial.println("Failed to open flash queue for batch write");
      return false;
    }
  }

  int written = 0;
  for (int i = 0; i < count; i++) {
    if (queueSize >= MAX_QUEUE_SIZE) {
      queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
      queueSize--;
    }

    file.seek(queueTail * sizeof(TelemetryData));
    size_t w = file.write((uint8_t*)&items[i], sizeof(TelemetryData));

    if (w != sizeof(TelemetryData)) {
      Serial.printf("Flash batch write failed at item %d\n", i);
      break;
    }

    queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
    queueSize++;
    written++;
  }

  file.close();
  flashWritesSinceLastSave += written;
  saveQueueMetadata();

  Serial.printf("Flushed %d items RAM->Flash | flash=%d ram=%d\n",
                written, queueSize, ramSize);
  return written > 0;
}

void flushRamToFlash(int count) {
  int actual = min(count, ramSize);
  TelemetryData batch[actual];
  for (int i = 0; i < actual; i++) {
    dequeueRam(batch[i]);
  }
  enqueueFlashBatch(batch, actual);
}

void enqueueTelemetry(const TelemetryData& data) {
  if (!enqueueRam(data)) {
    flushRamToFlash(RAM_BUFFER_SIZE / 2);
    enqueueRam(data);
  }
}

void fillJsonDoc(JsonDocument& doc, const TelemetryData& data) {
  doc["latitude"]  = data.latitude;
  doc["longitude"] = data.longitude;
  doc["datetime"]  = data.datetime;
  if (data.flags & FLAG_HAS_SPEED)   doc["speed"]       = data.speed;
  if (data.flags & FLAG_HAS_BEARING) doc["bearing"]      = data.bearing;
  if (data.flags & FLAG_HAS_TEMP) {
    doc["temperature"] = data.temperature;
    doc["humidity"]    = data.humidity;
  }
}

String telemetryToJson(const TelemetryData& data) {
  JsonDocument doc;
  fillJsonDoc(doc, data);
  String json;
  serializeJson(doc, json);
  return json;
}

bool sendBatchFromRam() {
  if (ramSize == 0 || !client.connected()) return false;

  int batchCount = min(BATCH_SIZE, ramSize);

  JsonDocument batchDoc;
  JsonArray arr = batchDoc.to<JsonArray>();

  int tempHead = ramHead;
  for (int i = 0; i < batchCount; i++) {
    JsonDocument itemDoc;
    fillJsonDoc(itemDoc, ramBuffer[tempHead]);
    arr.add(itemDoc);
    tempHead = (tempHead + 1) % RAM_BUFFER_SIZE;
  }

  char buffer[JSON_BUFFER_SIZE];
  size_t jsonSize = serializeJson(batchDoc, buffer, sizeof(buffer));
  if (jsonSize >= sizeof(buffer)) {
    Serial.println("RAM batch too large");
    return false;
  }

  String strID = String(device_id);
  strID.trim();
  String topic = "telemetry/" + strID + "/batch";

  if (client.publish(topic.c_str(), buffer, jsonSize)) {
    ramHead  = tempHead;
    ramSize -= batchCount;
    Serial.printf("\nRAM batch: %d sent, %d remaining\n", batchCount, ramSize);
    return true;
  }

  Serial.println("RAM batch publish failed");
  return false;
}

bool sendBatchFromFlash() {
  if (queueSize == 0 || !client.connected()) return false;

  int batchCount = min(BATCH_SIZE, queueSize);

  JsonDocument batchDoc;
  JsonArray arr = batchDoc.to<JsonArray>();

  TelemetryData batch[BATCH_SIZE];
  int tempHead   = queueHead;
  int actualRead = 0;

  File file = LittleFS.open(QUEUE_FILE, "r");
  if (!file) {
    Serial.println("Failed to open flash queue for reading");
    return false;
  }

  for (int i = 0; i < batchCount; i++) {
    file.seek(tempHead * sizeof(TelemetryData));
    size_t bytesRead = file.readBytes((char*)&batch[i], sizeof(TelemetryData));
    if (bytesRead != sizeof(TelemetryData)) break;

    actualRead++;
    tempHead = (tempHead + 1) % MAX_QUEUE_SIZE;

    JsonDocument itemDoc;
    fillJsonDoc(itemDoc, batch[i]);
    arr.add(itemDoc);
  }
  file.close();

  if (actualRead == 0) return false;

  char buffer[JSON_BUFFER_SIZE];
  size_t jsonSize = serializeJson(batchDoc, buffer, sizeof(buffer));
  if (jsonSize >= sizeof(buffer)) {
    Serial.println("Flash batch too large");
    return false;
  }

  String strID = String(device_id);
  strID.trim();
  String topic = "telemetry/" + strID + "/batch";

  Serial.printf("\nFlash batch: %d items (%d bytes)\n", actualRead, jsonSize);

  if (client.publish(topic.c_str(), buffer, jsonSize)) {
    queueHead  = (queueHead + actualRead) % MAX_QUEUE_SIZE;
    queueSize -= actualRead;
    saveQueueMetadata();
    Serial.printf("Flash remaining: %d items\n", queueSize);
    return true;
  }

  Serial.println("Flash batch publish failed");
  return false;
}

void loadConfig() {
  preferences.begin("gps-config", false);
  String server = preferences.getString("server", "192.168.1.100");
  String port   = preferences.getString("port",   "1883");
  String id     = preferences.getString("dev_id", "");
  String token  = preferences.getString("token",  "");
  preferences.end();

  strncpy(mqtt_server,  server.c_str(), 60); mqtt_server[60]  = '\0';
  strncpy(mqtt_port,    port.c_str(),   10); mqtt_port[10]    = '\0';
  strncpy(device_id,    id.c_str(),     32); device_id[32]    = '\0';
  strncpy(device_token, token.c_str(),  64); device_token[64] = '\0';

  Serial.println("\nCONFIG LOADED");
  Serial.printf("Server: '%s'\n", mqtt_server);
  Serial.printf("Port:   '%s'\n", mqtt_port);
  Serial.printf("ID:     '%s'\n", device_id);
}

void saveConfig() {
  preferences.begin("gps-config", false);
  preferences.putString("server",  mqtt_server);
  preferences.putString("port",    mqtt_port);
  preferences.putString("dev_id",  device_id);
  preferences.putString("token",   device_token);
  preferences.end();
  Serial.println("Config saved");
}

void saveFromPortal(WiFiManagerParameter* srv, WiFiManagerParameter* prt,
                    WiFiManagerParameter* id,  WiFiManagerParameter* tok) {
  const char* val;

  val = srv->getValue();
  if (val && strlen(val) > 0) {
    strncpy(mqtt_server, val, 60);
    mqtt_server[60] = '\0';
    for (int i = strlen(mqtt_server)-1; i >= 0 && mqtt_server[i]==' '; i--)
      mqtt_server[i] = '\0';
  }

  val = prt->getValue();
  if (val && strlen(val) > 0) {
    strncpy(mqtt_port, val, 10);
    mqtt_port[10] = '\0';
  }

  val = id->getValue();
  if (val && strlen(val) > 0) {
    strncpy(device_id, val, 32);
    device_id[32] = '\0';
    for (int i = strlen(device_id)-1; i >= 0 && device_id[i]==' '; i--)
      device_id[i] = '\0';
  }

  val = tok->getValue();
  if (val && strlen(val) > 0) {
    strncpy(device_token, val, 64);
    device_token[64] = '\0';
  }

  saveConfig();

  Serial.println("\nSAVED FROM PORTAL");
  Serial.printf("Server: '%s'\n", mqtt_server);
  Serial.printf("Port:   '%s'\n", mqtt_port);
  Serial.printf("ID:     '%s'\n", device_id);
}

void saveConfigCallback() {
  shouldSaveConfig = true;
}

String getISODateTime() {
  if (gps.date.isValid() && gps.time.isValid()) {
    char dt[25];
    sprintf(dt, "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    return String(dt);
  }
  unsigned long s = millis() / 1000;
  char dt[25];
  sprintf(dt, "1970-01-01T%02lu:%02lu:%02lu.000Z",
          (s/3600)%24, (s/60)%60, s%60);
  return String(dt);
}

void checkButton();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nGPS TRACKER STARTING");

  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_GPS,  OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);

  digitalWrite(LED_GPS,  LOW);
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_MQTT, LOW);

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GPS,  HIGH); delay(100); digitalWrite(LED_GPS,  LOW);
    digitalWrite(LED_WIFI, HIGH); delay(100); digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_MQTT, HIGH); delay(100); digitalWrite(LED_MQTT, LOW);
  }

  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  initFS();

  Serial.println("Initializing DHT11...");
  dht.begin();
  delay(2000);

  for (int i = 0; i < 3; i++) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      Serial.printf("DHT11 OK: %.1f°C, %.1f%%\n", t, h);
      dhtWorking = true;
      break;
    }
    Serial.printf("DHT11 attempt %d/3 failed\n", i+1);
    delay(2000);
  }
  if (!dhtWorking) Serial.println("DHT11 FAILED - continuing without temp");

  loadConfig();

  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", mqtt_server, 60);
  WiFiManagerParameter custom_mqtt_port  ("port",   "MQTT Port",      mqtt_port,   10);
  WiFiManagerParameter custom_device_id  ("dev_id", "Device ID",      device_id,   32);
  WiFiManagerParameter custom_device_token("token", "Device Token",   device_token,64);

  wm.setSaveConfigCallback(saveConfigCallback);
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_device_id);
  wm.addParameter(&custom_device_token);
  wm.setConfigPortalTimeout(180);

  Serial.println("Connecting to WiFi...");
  if (!wm.autoConnect("GPS-TRACKER-AP")) {
    Serial.println("Failed to connect, restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  if (shouldSaveConfig) {
    saveFromPortal(&custom_mqtt_server, &custom_mqtt_port,
                   &custom_device_id,   &custom_device_token);
  }

  int port = atoi(mqtt_port);
  Serial.printf("MQTT: %s:%d\n", mqtt_server, port);
  client.setServer(mqtt_server, port);
  client.setBufferSize(JSON_BUFFER_SIZE);
}

void reconnect() {
  while (!client.connected()) {
    checkButton();
    updateLEDs();

    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    String strID = String(device_id);
    strID.trim();
    String clientId = "GPS-" + strID;
    String username = "tracker-" + strID;

    Serial.printf("\nConnecting MQTT as '%s'...\n", clientId.c_str());

    if (client.connect(clientId.c_str(), username.c_str(), device_token)) {
      Serial.println("MQTT Connected");

      int batchCount = 0;
      while (ramSize > 0 && client.connected() && batchCount < 100) {
        while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
        if (!sendBatchFromRam()) break;
        batchCount++;
        updateLEDs();
        delay(50);
      }

      batchCount = 0;
      while (queueSize > 0 && client.connected() && batchCount < 100) {
        while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
        if (!sendBatchFromFlash()) break;
        batchCount++;
        updateLEDs();
        delay(100);
      }

      if (ramSize + queueSize > 0) {
        Serial.printf("Paused: %d RAM + %d Flash remaining\n",
                      ramSize, queueSize);
      }

    } else {
      Serial.printf("MQTT Failed RC=%d, retry in 5s\n", client.state());
      delay(5000);
    }
  }
}

void checkButton() {
  if (digitalRead(TRIGGER_PIN) == LOW) {
    delay(50);
    if (digitalRead(TRIGGER_PIN) == LOW) {
      Serial.println("\nCONFIG MODE");

      WiFiManagerParameter btn_mqtt_server("server", "MQTT Server IP", mqtt_server, 60);
      WiFiManagerParameter btn_mqtt_port  ("port",   "MQTT Port",      mqtt_port,   10);
      WiFiManagerParameter btn_device_id  ("dev_id", "Device ID",      device_id,   32);
      WiFiManagerParameter btn_device_token("token", "Device Token",   device_token,64);

      WiFiManager wm_button;
      wm_button.setSaveParamsCallback(saveConfigCallback);
      wm_button.addParameter(&btn_mqtt_server);
      wm_button.addParameter(&btn_mqtt_port);
      wm_button.addParameter(&btn_device_id);
      wm_button.addParameter(&btn_device_token);

      wm_button.setConfigPortalBlocking(false);
      wm_button.startConfigPortal("GPS-CONFIG");

      unsigned long start = millis();
      while (millis() - start < 180000) {
        wm_button.process();
        updateLEDs();
        if (shouldSaveConfig) {
          saveFromPortal(&btn_mqtt_server, &btn_mqtt_port,
                         &btn_device_id,   &btn_device_token);
          ESP.restart();
        }
        delay(10);
      }
      ESP.restart();
    }
  }
}

void loop() {
  checkButton();
  updateLEDs();

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastMsg = 0;
  if (millis() - lastMsg > TELEMETRY_INTERVAL) {
    lastMsg = millis();

    if (gps.location.isValid()) {
      TelemetryData data;
      data.latitude  = gps.location.lat();
      data.longitude = gps.location.lng();
      data.flags     = 0;

      String dt = getISODateTime();
      strncpy(data.datetime, dt.c_str(), 24);
      data.datetime[24] = '\0';

      if (gps.speed.isValid()) {
        data.speed  = gps.speed.kmph();
        data.flags |= FLAG_HAS_SPEED;
      }
      if (gps.course.isValid()) {
        data.bearing = gps.course.deg();
        data.flags  |= FLAG_HAS_BEARING;
      }
      if (dhtWorking) {
        float temperature = dht.readTemperature();
        float humidity    = dht.readHumidity();
        if (!isnan(temperature) && !isnan(humidity)) {
          data.temperature = round(temperature * 10) / 10.0;
          data.humidity    = round(humidity    * 10) / 10.0;
          data.flags      |= FLAG_HAS_TEMP;
        }
      }

      if (client.connected() && ramSize == 0 && queueSize == 0) {
        String json  = telemetryToJson(data);
        String strID = String(device_id);
        strID.trim();
        String topic = "telemetry/" + strID;

        if (client.publish(topic.c_str(), json.c_str())) {
          Serial.print(".");
        } else {
          Serial.print("x");
          enqueueTelemetry(data);
        }
      } else {
        enqueueTelemetry(data);
        Serial.print("q");

        if (client.connected() && ramSize >= BATCH_SIZE) {
          sendBatchFromRam();
        }
      }
    } else {
      Serial.print("-");
    }
  }

  if (client.connected()) {
    static unsigned long lastDrain = 0;
    if (millis() - lastDrain > 10000) {
      lastDrain = millis();
      if (ramSize   > 0) sendBatchFromRam();
      if (queueSize > 0) sendBatchFromFlash();
    }
  }
}
