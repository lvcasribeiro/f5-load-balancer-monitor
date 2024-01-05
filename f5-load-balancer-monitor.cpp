// Pinout:
const int ldr_pin = 32;
const int mq_pin = 33;
const int dht_pin = 4;

// Setup function:
void setup() {
    pinMode(ldr_pin, INPUT);
    pinMode(dht_pin, INPUT);
    pinMode(mq_pin, INPUT);

    Serial.begin(115200);
}
