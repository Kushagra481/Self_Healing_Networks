#define MQ2_PIN 34   // D34 (GPIO34)

int count = 0;
const int MAX_READINGS = 600;

// averaging for stability
int readMQ2() {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(MQ2_PIN);
    delay(20);
  }
  return sum / 5;
}

void setup() {
  Serial.begin(115200);
  delay(5000);  // warm-up
}

void loop() {
  if (count < MAX_READINGS) {

    int gas = readMQ2();

    Serial.print("MQ2:");
    Serial.println(gas);

    count++;

    delay(200);
  } else {
    while (true) delay(1000);
  }
}