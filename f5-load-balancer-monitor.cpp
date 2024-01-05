// Libraries import:
#include <WiFi.h>
#include <time.h>

// Pinout:
const int ldr_pin = 32;
const int mq_pin = 33;
const int dht_pin = 4;

// Local date and time capture and time zone correction:
#define NTP_SERVER "pool.ntp.br"
const int daylight_offset_sec = 0;
const long gmt_offset_sec = -3600*3;

// Wi-fi connection, network SSID and password:
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

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
}

// Loop function:
void loop() {
    unsigned long current_millis = millis();

    if (WiFi.status() == WL_CONNECTED) {
        timeCapture();
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
