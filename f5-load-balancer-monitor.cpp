// Libraries import:
#include <DHT.h>
#include <WiFi.h>
#include <time.h>

// Pinout:
const int ldr_pin = 32;
const int mq_pin = 33;
const int dht_pin = 4;

// DHT setup:
#define dht_type DHT22
DHT dht(dht_pin, dht_type);

// MQ-135 setup:
#define rl_value 20
#define ro_clean_air_factor 9.83

// Time management:
unsigned long previous_millis = 0;
const long interval = 5000;

// Local date and time capture and time zone correction:
#define NTP_SERVER "pool.ntp.br"
const int daylight_offset_sec = 0;
const long gmt_offset_sec = -3600*3;

// Wi-fi connection, network SSID and password:
#define WIFI_SSID "CLARO_2GDD5E65"
#define WIFI_PASSWORD "12DD5E65"

// Global variables:
const int debug = 2;

char current_date[15];
char current_time[15];

char current_hour[3];
char current_minute[3];
char current_week_day[10];

// Setup function:
void setup() {
    pinMode(ldr_pin, INPUT);
    pinMode(dht_pin, INPUT);
    pinMode(mq_pin, INPUT);

    Serial.begin(115200);

    configTime(gmt_offset_sec, daylight_offset_sec, NTP_SERVER);
    connectToWifi();

    dht.begin();
}

// Loop function:
void loop() {
    unsigned long current_millis = millis();

    if (WiFi.status() == WL_CONNECTED) {
        timeCapture();

        if (current_millis - previous_millis >= interval) {
            previous_millis = current_millis;

            int light_analog_value = analogRead(ldr_pin);
            float humidity = dht.readHumidity();
            float temperature = dht.readTemperature();
            float heat_index = dht.computeHeatIndex(temperature, humidity, false);

            float ppm = gasPercentage();

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
                Serial.println(" ppm");
            }
        }
    } else {
        if (debug >= 1) {
            Serial.println("- Loss of Wi-Fi connection.");
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
