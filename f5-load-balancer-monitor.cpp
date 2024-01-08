// Libraries import:
#include <Adafruit_Sensor.h>
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <WiFi.h>
#include <time.h>

// Pinout:
const int ldr_pin = 32;
const int mq_pin = 33;
const int ky_pin = 35;
const int dht_pin = 4;

#define HEARTBEAT_PIN 15
#define SENT_PIN 21
#define FAIL_PIN 5

// DHT-22 setup:
#define dht_type DHT22
DHT dht(dht_pin, dht_type);

// MQ-135 setup:
#define rl_value 20
#define ro_clean_air_factor 9.83

// Time management:
unsigned long realtime_millis = 0;
unsigned long json_millis = 0;
const long realtime_interval = 5*1000;
const long json_interval = 5*60*1000;

// Local date and time capture and time zone correction:
#define NTP_SERVER "pool.ntp.br"
const int daylight_offset_sec = 0;
const long gmt_offset_sec = -3600*3;

// Wi-Fi connection, network SSID and password:
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// Cloud database connection - Firebase, host and auth:
#define FIREBASE_AUTH "your_firebase_auth"
#define FIREBASE_HOST "your_firebase_host"

FirebaseData firebaseData;
FirebaseJson json_measured_variables;

// Global variables:
const int debug = 2;

char current_date[15];
char current_time[15];

char current_hour[3];
char current_minute[3];
char current_week_day[10];

String ip_address = "0.0.0.0";
String board_status = "Offline";
String cloud_status = "Connected";

// Setup function:
void setup() {
    pinMode(ldr_pin, INPUT);
    pinMode(dht_pin, INPUT);
    pinMode(ky_pin, INPUT);
    pinMode(mq_pin, INPUT);
    pinMode(HEARTBEAT_PIN, OUTPUT);
    pinMode(SENT_PIN, OUTPUT);
    pinMode(FAIL_PIN, OUTPUT);

    Serial.begin(115200);

    configTime(gmt_offset_sec, daylight_offset_sec, NTP_SERVER);
    connectToWifi();

    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);

    dht.begin();
}

// Loop function:
void loop() {
    unsigned long current_millis = millis();

    if (WiFi.status() == WL_CONNECTED) {
        timeCapture();

        int light_analog_value = analogRead(ldr_pin);
        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        float heat_index = dht.computeHeatIndex(temperature, humidity, false);

        float ppm = gasPercentage();
        float noise = soundDecibels();

        temperature = round(temperature * 100.0) / 100.0;
        humidity = round(humidity * 100.0) / 100.0;
        heat_index = round(heat_index * 100.0) / 100.0;
        ppm = round(ppm * 100.0) / 100.0;
        noise = round(noise * 100.0) / 100.0;

        if (current_millis - realtime_millis >= realtime_interval) {
            realtime_millis = current_millis;

            if (debug >= 2) {
                Serial.print("- ");
                Serial.print(current_time);
                Serial.print(" - Light analog value: ");
                Serial.print(light_analog_value);
                Serial.print(" - Temperature: ");
                Serial.print(temperature);
                Serial.print(" °C - Humidity: ");
                Serial.print(humidity);
                Serial.print("% - Heat Index: ");
                Serial.print(heat_index);
                Serial.print(" °C - Gas Concentration: ");
                Serial.print(ppm);
                Serial.print(" ppm - Sound Intensity: ");
                Serial.print(noise);
                Serial.println(" dB");
            }

            firebaseRealtimeSync(temperature, humidity, heat_index, light_analog_value, ppm, noise, ip_address, board_status, cloud_status);
        }

        if (current_millis - json_millis >= json_interval) {
            json_millis = current_millis;

            firebaseJsonSync(temperature, humidity, heat_index, light_analog_value, ppm, noise, current_time, current_date);
        }
    } else {
        if (debug >= 1) {
            Serial.println("- Loss of Wi-Fi connection.");

            board_status = "Offline";
        }

        connectToWifi();
    }
}

// Time management function:
void timeCapture() {
    struct tm time_info;

    if (!getLocalTime(&time_info)) {
        if (debug >= 1) {
            Serial.println("- Failed to capture NTP server data.");
        }

        configTime(gmt_offset_sec, daylight_offset_sec, NTP_SERVER);
        
        return;
    }

    strftime(current_date, 15, "%e/%m/%Y", &time_info);
    strftime(current_time, 15, "%H:%M:%S", &time_info);

    strftime(current_hour, 3, "%H", &time_info);
    strftime(current_minute, 3, "%M", &time_info);
    strftime(current_week_day, 10, "%A", &time_info);
}

// Wi-fi connection function:
void connectToWifi() {
    if (debug >= 1) {
        Serial.print("- Connecting to the Wi-Fi network...");
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long wifi_millis = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - wifi_millis < 5000) {
        if (debug >= 1) {
            Serial.print(".");
        }
        
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (debug >= 1) {
            Serial.print("\n- Connected to Wi-Fi network, IP address: ");
            Serial.println(WiFi.localIP());

            ip_address = WiFi.localIP().toString();
            board_status = "Online";
        }
    } else {
        if (debug >= 1) {
            Serial.println("\n- Failed to connect to Wi-Fi.");
        }
    }
}

// Gas concentration function:
float gasPercentage() {
    int adc_value = analogRead(mq_pin);

    float voltage = adc_value * (3.3 / 1023.0);
    float sensor_resistance = ((3.3 - voltage) / voltage) * rl_value;
    float ratio = sensor_resistance / ro_clean_air_factor;
    float ppm = 10000.0 * pow(ratio, -2.11);

    return ppm;
}

// Decibels sound conversion function:
float soundDecibels() {
    int adc_value = analogRead(ky_pin);

    float voltage_ratio = static_cast<float>(adc_value) / 3.3;

    if (voltage_ratio <= 0) {
        return -1.0;
    }

    float dB = 20 * log10(voltage_ratio);

    return dB;
}

// Firebase realtime sync function:
void firebaseRealtimeSync(float temperature, float humidity, float heat_index, int light_analog_value, float ppm, float noise, String ip_address, String board_status, String cloud_status) {
    if (Firebase.get(firebaseData, "/measured_variables")) {
        Firebase.set(firebaseData, "measured_variables/temperature", temperature);
        Firebase.set(firebaseData, "measured_variables/humidity", humidity);
        Firebase.set(firebaseData, "measured_variables/heat_index", heat_index);
        Firebase.set(firebaseData, "measured_variables/luminosity", light_analog_value);
        Firebase.set(firebaseData, "measured_variables/gas_concentration", ppm);
        Firebase.set(firebaseData, "measured_variables/sound_noise", noise);

        Firebase.set(firebaseData, "settings/ip_address", ip_address);
        Firebase.set(firebaseData, "settings/board_status", board_status);
        Firebase.set(firebaseData, "settings/cloud_status", cloud_status);

        digitalWrite(HEARTBEAT_PIN, HIGH);
        cloud_status = "Connected";

        unsigned long led_heartbeat_millis = millis();
        bool led_heartbeat_on = true;

        while (led_heartbeat_on) {
            if (millis() - led_heartbeat_millis >= 750) { 
                digitalWrite(HEARTBEAT_PIN, LOW);
                led_heartbeat_on = false;
            }
        }
    } else {
        if (debug >= 1) {
            Serial.print("- Error updating cloud data: [FIREBASE LATENCY][ttl] - ");
            Serial.print(firebaseData.errorReason());
            Serial.println(".");
        }

        cloud_status = "Disconnected";
    }
}

// Firebase JSON sync function:
void firebaseJsonSync(float temperature, float humidity, float heat_index, int light_analog_value, float ppm, float noise, String current_time, String current_date) {
    json_measured_variables.set("temperature", temperature);
    json_measured_variables.set("humidity", humidity);
    json_measured_variables.set("heat_index", heat_index);
    json_measured_variables.set("luminosity", light_analog_value);
    json_measured_variables.set("gas_concentration", ppm);
    json_measured_variables.set("sound_noise", noise);
    json_measured_variables.set("timestamp", current_time);
    json_measured_variables.set("date", current_date);

    if (Firebase.pushJSON(firebaseData, "/json_storage", json_measured_variables)) {
        digitalWrite(SENT_PIN, HIGH);
        cloud_status = "Connected";

        unsigned long led_sent_millis = millis();
        bool led_sent_on = true;

        while (led_sent_on) {
            if (millis() - led_sent_millis >= 750) { 
                digitalWrite(SENT_PIN, LOW);
                led_sent_on = false;
            }
        }
    } else {
        if (debug >= 1) {
            Serial.print("- Error uploading JSON to cloud database: [FIREBASE LATENCY][ttl] - ");
            Serial.print(firebaseData.errorReason());
            Serial.println(".");
        }

        digitalWrite(FAIL_PIN, HIGH);
        cloud_status = "Disconnected";

        unsigned long led_fail_millis = millis();
        bool led_fail_on = true;

        while (led_fail_on) {
            if (millis() - led_fail_millis >= 750) { 
                digitalWrite(FAIL_PIN, LOW);
                led_fail_on = false;
            }
        }
    }
}
