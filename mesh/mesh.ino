#include <WiFi.h>
#include <WiFiUdp.h>

#define WIFI_SSID     "OnePlus Nord CE5 p2jr"
#define WIFI_PASS     "vvci6957"

#define UDP_PORT      5005
#define BROADCAST_IP  IPAddress(255,255,255,255)

#define NODE_ID       5   // CHANGE THIS: 1..5

#define MSG_HELLO  1
#define MSG_DATA   2

WiFiUDP udp;

struct MeshPacket {
  uint8_t type;
  uint8_t src_id;
  uint8_t dst_id;
  uint8_t ttl;
  uint32_t seq;
  char payload[64];
};

struct Neighbor {
  uint8_t id;
  IPAddress ip;
};

Neighbor neighbors[10];
int neighborCount = 0;

uint32_t seqCounter = 0;

/* ---------------- Utility ---------------- */

void addNeighbor(uint8_t id, IPAddress ip) {
  for (int i = 0; i < neighborCount; i++) {
    if (neighbors[i].id == id) return;
  }
  neighbors[neighborCount++] = {id, ip};
  Serial.printf("Neighbor added: Node %d @ %s\n", id, ip.toString().c_str());
}

/* ---------------- Send ---------------- */

void sendPacket(IPAddress ip, MeshPacket &pkt) {
  udp.beginPacket(ip, UDP_PORT);
  udp.write((uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
}

/* ---------------- Receive ---------------- */

void handlePacket(MeshPacket &pkt, IPAddress senderIP) {

  if (pkt.src_id == NODE_ID) return; // ignore own packets

  addNeighbor(pkt.src_id, senderIP);

  if (pkt.type == MSG_HELLO) {
    Serial.printf("HELLO from Node %d\n", pkt.src_id);
    return;
  }

  if (pkt.type == MSG_DATA) {

    if (pkt.dst_id == NODE_ID) {
      Serial.printf("DATA RECEIVED from %d: %s\n",
                    pkt.src_id, pkt.payload);
      return;
    }

    if (pkt.ttl == 0) return;

    pkt.ttl--;

    for (int i = 0; i < neighborCount; i++) {
      if (neighbors[i].id != pkt.src_id) {
        sendPacket(neighbors[i].ip, pkt);
      }
    }
  }
}

/* ---------------- Setup ---------------- */

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
}

/* ---------------- Loop ---------------- */

void loop() {

  // Receive packets
  int size = udp.parsePacket();
  if (size == sizeof(MeshPacket)) {
    MeshPacket pkt;
    udp.read((uint8_t*)&pkt, sizeof(pkt));
    handlePacket(pkt, udp.remoteIP());
  }

  // Periodic HELLO
  static unsigned long lastHello = 0;
  if (millis() - lastHello > 5000) {
    lastHello = millis();

    MeshPacket hello{};
    hello.type = MSG_HELLO;
    hello.src_id = NODE_ID;

    sendPacket(BROADCAST_IP, hello);
  }

  // Periodic DATA test (Node 1 → Node 5)
  static unsigned long lastData = 0;
  if (NODE_ID == 1 && millis() - lastData > 8000) {
    lastData = millis();

    MeshPacket data{};
    data.type = MSG_DATA;
    data.src_id = NODE_ID;
    data.dst_id = 5;
    data.ttl = 5;
    data.seq = seqCounter++;
    strcpy(data.payload, "Hello through UDP mesh");

    for (int i = 0; i < neighborCount; i++) {
      sendPacket(neighbors[i].ip, data);
    }
  }
}
