#include <PubSubClient.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFiManager.h> 
#include <ArduinoJson.h>
#include <Preferences.h>
#include <DHT.h>
#include <SPIFFS.h>

#define TRIGGER_PIN 0
#define DHT_PIN 21
#define DHT_TYPE DHT11

#define LED_GPS  22
#define LED_WIFI 23
#define LED_MQTT 13

#define MAX_QUEUE_SIZE 5000
#define BATCH_SIZE 20
#define TELEMETRY_INTERVAL 5000
#define MAX_STORAGE_KB 512
#define QUEUE_FILE "/queue.dat"

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
#define RX_PIN 18
#define TX_PIN 19

DHT dht(DHT_PIN, DHT_TYPE);

WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

char mqtt_server[61] = "";
char mqtt_port[11] = "";
char device_id[33] = "";
char device_token[65] = "";

bool shouldSaveConfig = false;
bool dhtWorking = false;
WiFiManager wm; 

struct TelemetryData {
  float latitude;
  float longitude;
  float speed;
  float bearing;
  float temperature;
  float humidity;
  char datetime[25];
  uint8_t flags;
} __attribute__((packed));

#define FLAG_HAS_SPEED    0x01
#define FLAG_HAS_BEARING  0x02
#define FLAG_HAS_TEMP     0x04

int queueHead = 0;
int queueTail = 0;
int queueSize = 0;

unsigned long lastGpsValid = 0;
unsigned long ledBlinkTimer = 0;
bool ledBlinkState = false;

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
  
  if (client.connected()) {
    if (queueSize > 0) {
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

void saveConfigCallback() {
  shouldSaveConfig = true;
}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  
  Serial.println("SPIFFS Mounted");
  
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  
  Serial.printf("Total: %d KB\n", totalBytes / 1024);
  Serial.printf("Used: %d KB\n", usedBytes / 1024);
  Serial.printf("Free: %d KB\n", (totalBytes - usedBytes) / 1024);
  
  loadQueueMetadata();
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
  
  Serial.printf("Queue loaded: %d items (head=%d, tail=%d)\n", 
                queueSize, queueHead, queueTail);
}

void saveQueueMetadata() {
  preferences.begin("queue", false);
  preferences.putInt("head", queueHead);
  preferences.putInt("tail", queueTail);
  preferences.putInt("size", queueSize);
  preferences.end();
}

bool checkStorageSpace() {
  size_t usedBytes = SPIFFS.usedBytes();
  size_t totalBytes = SPIFFS.totalBytes();
  size_t freeBytes = totalBytes - usedBytes;
  
  if (freeBytes < 10240) {
    Serial.printf("Low storage: %d KB free\n", freeBytes / 1024);
    return false;
  }
  
  size_t maxAllowedBytes = MAX_STORAGE_KB * 1024;
  if (usedBytes > maxAllowedBytes) {
    Serial.printf("Storage limit exceeded: %d KB > %d KB\n", 
                  usedBytes / 1024, MAX_STORAGE_KB);
    return false;
  }
  
  return true;
}

bool enqueueTelemetry(const TelemetryData& data) {
  if (queueSize >= MAX_QUEUE_SIZE) {
    Serial.printf("Queue full (%d), dropping oldest\n", MAX_QUEUE_SIZE);
    
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    queueSize--;
  }
  
  if (!checkStorageSpace()) {
    Serial.println("Storage full, dropping oldest entries");
    
    for (int i = 0; i < 10 && queueSize > 0; i++) {
      queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
      queueSize--;
    }
  }
  
  File file = SPIFFS.open(QUEUE_FILE, "r+");
  if (!file) {
    file = SPIFFS.open(QUEUE_FILE, "w+");
    if (!file) {
      Serial.println("Failed to open queue file");
      return false;
    }
  }
  
  size_t offset = queueTail * sizeof(TelemetryData);
  file.seek(offset);
  
  size_t written = file.write((uint8_t*)&data, sizeof(TelemetryData));
  file.close();
  
  if (written != sizeof(TelemetryData)) {
    Serial.println("Failed to write telemetry");
    return false;
  }
  
  queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
  queueSize++;
  
  if (queueSize % 10 == 0) {
    saveQueueMetadata();
  }
  
  if (queueSize % 100 == 0) {
    Serial.printf("Queue size: %d items (%.1f KB)\n", 
                  queueSize, 
                  (queueSize * sizeof(TelemetryData)) / 1024.0);
  }
  
  return true;
}

void clearQueue() {
  queueHead = queueTail = queueSize = 0;
  saveQueueMetadata();
  SPIFFS.remove(QUEUE_FILE);
  Serial.println("Queue cleared");
}

String telemetryToJson(const TelemetryData& data) {
  JsonDocument doc;
  
  doc["latitude"] = data.latitude;
  doc["longitude"] = data.longitude;
  doc["datetime"] = data.datetime;
  
  if (data.flags & FLAG_HAS_SPEED) {
    doc["speed"] = data.speed;
  }
  
  if (data.flags & FLAG_HAS_BEARING) {
    doc["bearing"] = data.bearing;
  }
  
  if (data.flags & FLAG_HAS_TEMP) {
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
  }
  
  String json;
  serializeJson(doc, json);
  return json;
}

bool sendBatch() {
  if (queueSize == 0 || !client.connected()) {
    return false;
  }
  
  int batchSize = min(BATCH_SIZE, queueSize);
  
  JsonDocument batchDoc;
  JsonArray arr = batchDoc.to<JsonArray>();
  
  TelemetryData tempQueue[BATCH_SIZE];
  int actualRead = 0;
  
  int tempHead = queueHead;
  
  for (int i = 0; i < batchSize; i++) {
    File file = SPIFFS.open(QUEUE_FILE, "r");
    if (!file) {
      Serial.println("Failed to open queue for reading");
      break;
    }
    
    size_t offset = tempHead * sizeof(TelemetryData);
    file.seek(offset);
    
    size_t read = file.readBytes((char*)&tempQueue[i], sizeof(TelemetryData));
    file.close();
    
    if (read != sizeof(TelemetryData)) {
      Serial.println("Failed to read telemetry");
      break;
    }
    
    actualRead++;
    tempHead = (tempHead + 1) % MAX_QUEUE_SIZE;
    
    JsonDocument itemDoc;
    itemDoc["latitude"] = tempQueue[i].latitude;
    itemDoc["longitude"] = tempQueue[i].longitude;
    itemDoc["datetime"] = tempQueue[i].datetime;
    
    if (tempQueue[i].flags & FLAG_HAS_SPEED) {
      itemDoc["speed"] = tempQueue[i].speed;
    }
    
    if (tempQueue[i].flags & FLAG_HAS_BEARING) {
      itemDoc["bearing"] = tempQueue[i].bearing;
    }
    
    if (tempQueue[i].flags & FLAG_HAS_TEMP) {
      itemDoc["temperature"] = tempQueue[i].temperature;
      itemDoc["humidity"] = tempQueue[i].humidity;
    }
    
    arr.add(itemDoc);
  }
  
  if (actualRead == 0) {
    return false;
  }
  
  char buffer[4096];
  size_t jsonSize = serializeJson(batchDoc, buffer, sizeof(buffer));
  
  if (jsonSize >= sizeof(buffer)) {
    Serial.println("Batch too large");
    return false;
  }
  
  String strID = String(device_id);
  strID.trim();
  String topic = "telemetry/" + strID + "/batch";
  
  Serial.printf("\nBatch: %d items (%d bytes)\n", actualRead, jsonSize);
  
  if (client.publish(topic.c_str(), buffer, jsonSize)) {
    Serial.println("Published successfully");
    
    queueHead = (queueHead + actualRead) % MAX_QUEUE_SIZE;
    queueSize -= actualRead;
    saveQueueMetadata();
    
    Serial.printf("Remaining: %d items\n", queueSize);
    return true;
  } else {
    Serial.println("Publish failed, keeping in queue");
    return false;
  }
}

void loadConfig() {
  preferences.begin("gps-config", false);
  
  String server = preferences.getString("server", "192.168.1.100");
  String port = preferences.getString("port", "1883");
  String id = preferences.getString("dev_id", "");
  String token = preferences.getString("token", "");
  
  strncpy(mqtt_server, server.c_str(), 60);
  mqtt_server[60] = '\0';
  
  strncpy(mqtt_port, port.c_str(), 10);
  mqtt_port[10] = '\0';
  
  strncpy(device_id, id.c_str(), 32);
  device_id[32] = '\0';
  
  strncpy(device_token, token.c_str(), 64);
  device_token[64] = '\0';
  
  preferences.end();
  
  Serial.println("\nCONFIG LOADED");
  Serial.printf("Server: '%s'\n", mqtt_server);
  Serial.printf("Port: '%s'\n", mqtt_port);
  Serial.printf("Device ID: '%s'\n", device_id);
  Serial.printf("Token: '%s'\n", device_token);
  Serial.println();
}

void saveConfig() {
  preferences.begin("gps-config", false);
  
  preferences.putString("server", mqtt_server);
  preferences.putString("port", mqtt_port);
  preferences.putString("dev_id", device_id);
  preferences.putString("token", device_token);
  
  preferences.end();
  
  Serial.println("Config saved to NVS");
}

void saveFromPortal(WiFiManagerParameter* srv, WiFiManagerParameter* prt, 
                    WiFiManagerParameter* id, WiFiManagerParameter* tok) {
  const char* val;
  
  Serial.println("\nReading values from portal");
  
  val = srv->getValue();
  if (val != nullptr && strlen(val) > 0) {
    strncpy(mqtt_server, val, 60);
    mqtt_server[60] = '\0';
    for (int i = strlen(mqtt_server) - 1; i >= 0 && mqtt_server[i] == ' '; i--) {
      mqtt_server[i] = '\0';
    }
  }
  
  val = prt->getValue();
  if (val != nullptr && strlen(val) > 0) {
    strncpy(mqtt_port, val, 10);
    mqtt_port[10] = '\0';
  }
  
  val = id->getValue();
  if (val != nullptr && strlen(val) > 0) {
    strncpy(device_id, val, 32);
    device_id[32] = '\0';
    for (int i = strlen(device_id) - 1; i >= 0 && device_id[i] == ' '; i--) {
      device_id[i] = '\0';
    }
  }
  
  val = tok->getValue();
  if (val != nullptr && strlen(val) > 0) {
    strncpy(device_token, val, 64);
    device_token[64] = '\0';
  }
  
  saveConfig();
  
  Serial.println("\nSAVED VALUES");
  Serial.printf("Server: '%s'\n", mqtt_server);
  Serial.printf("Port: '%s'\n", mqtt_port);
  Serial.printf("Device ID: '%s'\n", device_id);
  Serial.printf("Token: '%s'\n", device_token);
  Serial.println();
}

void checkButton();

String getISODateTime() {
  if (gps.date.isValid() && gps.time.isValid()) {
    char datetime[25];
    sprintf(datetime, "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
            gps.date.year(),
            gps.date.month(),
            gps.date.day(),
            gps.time.hour(),
            gps.time.minute(),
            gps.time.second());
    return String(datetime);
  }
  
  unsigned long seconds = millis() / 1000;
  char datetime[25];
  sprintf(datetime, "1970-01-01T%02lu:%02lu:%02lu.000Z",
          (seconds / 3600) % 24,
          (seconds / 60) % 60,
          seconds % 60);
  return String(datetime);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nGPS TRACKER STARTING");
  
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_GPS, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);
  
  digitalWrite(LED_GPS, LOW);
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_MQTT, LOW);
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GPS, HIGH);
    delay(100);
    digitalWrite(LED_GPS, LOW);
    digitalWrite(LED_WIFI, HIGH);
    delay(100);
    digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_MQTT, HIGH);
    delay(100);
    digitalWrite(LED_MQTT, LOW);
  }
  
  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  initSPIFFS();
  
  Serial.println("Initializing DHT11...");
  dht.begin();
  delay(2000);
  
  for (int i = 0; i < 3; i++) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    
    if (!isnan(t) && !isnan(h)) {
      Serial.printf("DHT11 OK: %.1f°C, %.1f%% RH\n", t, h);
      dhtWorking = true;
      break;
    }
    
    Serial.printf("DHT11 attempt %d/3 failed\n", i + 1);
    delay(2000);
  }
  
  if (!dhtWorking) {
    Serial.println("DHT11 FAILED - continuing without temperature");
  }
  
  loadConfig();
  
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", mqtt_server, 60);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 10);
  WiFiManagerParameter custom_device_id("dev_id", "Device ID", device_id, 32);
  WiFiManagerParameter custom_device_token("token", "Device Token", device_token, 64);
  
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
    Serial.println("\nSaving new config from setup");
    saveFromPortal(&custom_mqtt_server, &custom_mqtt_port, 
                   &custom_device_id, &custom_device_token);
  }
  
  int port = atoi(mqtt_port);
  Serial.printf("Setting up MQTT: %s:%d\n", mqtt_server, port);
  client.setServer(mqtt_server, port);
  client.setBufferSize(4096);
}

void reconnect() {
  while (!client.connected()) {
    checkButton();
    updateLEDs();
    
    String strID = String(device_id);
    strID.trim();
    
    String clientId = "GPS-" + strID;
    String username = "tracker-" + strID;
    
    Serial.printf("\nConnecting to MQTT as '%s'...\n", clientId.c_str());
    Serial.printf("Server: %s:%s\n", mqtt_server, mqtt_port);
    
    if (client.connect(clientId.c_str(), username.c_str(), device_token)) {
      Serial.println("MQTT Connected");
      
      if (queueSize > 0) {
        Serial.printf("Processing queue: %d items\n", queueSize);
        
        int maxBatches = 100;
        int batchCount = 0;
        
        while (queueSize > 0 && client.connected() && batchCount < maxBatches) {
          if (!sendBatch()) {
            Serial.println("Batch failed, will retry later");
            break;
          }
          batchCount++;
          updateLEDs();
          delay(100);
        }
        
        if (queueSize > 0) {
          Serial.printf("Paused batch sending, %d items remaining\n", queueSize);
        }
      }
    } else {
      Serial.printf("MQTT Failed! RC=%d\n", client.state());
      Serial.println("Retrying in 5s...");
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
      WiFiManagerParameter btn_mqtt_port("port", "MQTT Port", mqtt_port, 10);
      WiFiManagerParameter btn_device_id("dev_id", "Device ID", device_id, 32);
      WiFiManagerParameter btn_device_token("token", "Device Token", device_token, 64);
      
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
                         &btn_device_id, &btn_device_token);
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
      data.latitude = gps.location.lat();
      data.longitude = gps.location.lng();
      data.flags = 0;
      
      String datetime = getISODateTime();
      strncpy(data.datetime, datetime.c_str(), 24);
      data.datetime[24] = '\0';
      
      if (gps.speed.isValid()) {
        data.speed = gps.speed.kmph();
        data.flags |= FLAG_HAS_SPEED;
      }
      
      if (gps.course.isValid()) {
        data.bearing = gps.course.deg();
        data.flags |= FLAG_HAS_BEARING;
      }
      
      if (dhtWorking) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        
        if (!isnan(temperature) && !isnan(humidity)) {
          data.temperature = round(temperature * 10) / 10.0;
          data.humidity = round(humidity * 10) / 10.0;
          data.flags |= FLAG_HAS_TEMP;
        }
      }
      
      if (client.connected() && queueSize == 0) {
        String json = telemetryToJson(data);
        
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
        
        if (client.connected() && queueSize >= BATCH_SIZE) {
          sendBatch();
        }
      }
    } else {
      Serial.print("-");
    }
  }
  
  if (client.connected() && queueSize > 0) {
    static unsigned long lastBatchSend = 0;
    if (millis() - lastBatchSend > 10000) {
      lastBatchSend = millis();
      sendBatch();
    }
  }
}
