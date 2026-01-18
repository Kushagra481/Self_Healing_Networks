#include <WiFi.h>
#include <WiFiUdp.h>

/* ================= CONFIG ================= */
#define WIFI_SSID     "SSID"
#define WIFI_PASS     "PASSWORD"
#define UDP_PORT      5005
#define BROADCAST_IP  IPAddress(255,255,255,255)

#define NODE_ID       5     // CHANGE PER ESP32 (1.. 5)
#define MAX_HOPS      10

#define MAX_ROUTES    10
#define MAX_RREQ_CACHE 20
#define MAX_DATA_CACHE 20

/* ================= MESSAGE TYPES ================= */
#define MSG_RREQ  1
#define MSG_RREP  2
#define MSG_DATA  3

WiFiUDP udp;

/* ================= STRUCTURES ================= */
struct MeshPacket {
  uint8_t type;
  uint8_t src_id;
  uint8_t dst_id;
  uint8_t last_hop;
  uint8_t hop_count;
  uint32_t seq;
  char payload[64];
};

struct RouteEntry {
  uint8_t dst_id;
  uint8_t next_hop;
  IPAddress next_ip;
  uint8_t hop_count;
  uint32_t seq;
  bool active;
};

struct RREQCache {
  uint8_t src;
  uint32_t seq;
  unsigned long ts;
};

struct DataCache {
  uint8_t src;
  uint32_t seq;
  unsigned long ts;
};

/* ================= GLOBALS ================= */
RouteEntry routes[MAX_ROUTES];
int routeCount = 0;

RREQCache rreqCache[MAX_RREQ_CACHE];
int rreqCount = 0;

DataCache dataCache[MAX_DATA_CACHE];
int dataCount = 0;

uint32_t seqCounter = 0;
bool dataSent = false;

/* ================= UTIL ================= */
RouteEntry* findRoute(uint8_t dst) {
  for (int i = 0; i < routeCount; i++)
    if (routes[i].dst_id == dst && routes[i].active)
      return &routes[i];
  return nullptr;
}

void addRoute(uint8_t dst, uint8_t nextHop, IPAddress ip, uint8_t hops, uint32_t seq) {
  for (int i = 0; i < routeCount; i++) {
    if (routes[i].dst_id == dst) {
      if (seq >= routes[i].seq) {
        routes[i] = {dst, nextHop, ip, hops, seq, true};
      }
      return;
    }
  }
  if (routeCount < MAX_ROUTES)
    routes[routeCount++] = {dst, nextHop, ip, hops, seq, true};
}

bool seenRREQ(uint8_t src, uint32_t seq) {
  unsigned long now = millis();
  for (int i = 0; i < rreqCount; i++) {
    if (now - rreqCache[i].ts > 5000) {
      rreqCache[i] = rreqCache[--rreqCount];
      i--;
    }
  }
  for (int i = 0; i < rreqCount; i++)
    if (rreqCache[i].src == src && rreqCache[i]. seq == seq)
      return true;

  if (rreqCount < MAX_RREQ_CACHE)
    rreqCache[rreqCount++] = {src, seq, now};

  return false;
}

bool seenData(uint8_t src, uint32_t seq) {
  unsigned long now = millis();
  for (int i = 0; i < dataCount; i++) {
    if (now - dataCache[i].ts > 10000) {
      dataCache[i] = dataCache[--dataCount];
      i--;
    }
  }
  for (int i = 0; i < dataCount; i++)
    if (dataCache[i].src == src && dataCache[i]. seq == seq)
      return true;

  if (dataCount < MAX_DATA_CACHE)
    dataCache[dataCount++] = {src, seq, now};

  return false;
}

/* ================= SEND ================= */
void sendPkt(IPAddress ip, MeshPacket &pkt) {
  udp.beginPacket(ip, UDP_PORT);
  udp.write((uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
}

/* ================= RECEIVE ================= */
void handlePacket(MeshPacket &pkt, IPAddress senderIP) {
  if (pkt.hop_count > MAX_HOPS || pkt.src_id == NODE_ID)
    return;

  if (pkt.type == MSG_RREQ) {
    if (seenRREQ(pkt.src_id, pkt.seq)) return;

    addRoute(pkt.src_id, pkt. last_hop, senderIP, pkt.hop_count + 1, pkt.seq);

    if (pkt. dst_id == NODE_ID) {
      // We are the destination, send RREP back
      MeshPacket rrep{};
      rrep.type = MSG_RREP;
      rrep.src_id = NODE_ID;
      rrep.dst_id = pkt. src_id;
      rrep. last_hop = NODE_ID;
      rrep.hop_count = 0;
      rrep.seq = pkt.seq;
      snprintf(rrep.payload, sizeof(rrep.payload), "Route found to node %d", NODE_ID);

      RouteEntry* route = findRoute(pkt.src_id);
      if (route) {
        sendPkt(route->next_ip, rrep);
        Serial.printf("[RREP] Sent to node %d via %d\n", pkt.src_id, route->next_hop);
      }
    } else {
      // Forward RREQ
      pkt.last_hop = NODE_ID;
      pkt. hop_count++;
      sendPkt(BROADCAST_IP, pkt);
      Serial.printf("[RREQ] Forwarded from %d to %d (hops: %d)\n", pkt. src_id, pkt.dst_id, pkt.hop_count);
    }
  }
  else if (pkt.type == MSG_RREP) {
    addRoute(pkt. src_id, pkt.last_hop, senderIP, pkt.hop_count + 1, pkt.seq);

    if (pkt.dst_id == NODE_ID) {
      Serial.printf("[RREP] Received route to node %d (hops: %d)\n", pkt.src_id, pkt.hop_count + 1);
    } else {
      // Forward RREP toward destination
      RouteEntry* route = findRoute(pkt.dst_id);
      if (route) {
        pkt.last_hop = NODE_ID;
        pkt.hop_count++;
        sendPkt(route->next_ip, pkt);
        Serial.printf("[RREP] Forwarded to node %d via %d\n", pkt.dst_id, route->next_hop);
      }
    }
  }
  else if (pkt.type == MSG_DATA) {
    if (seenData(pkt.src_id, pkt.seq)) return;

    if (pkt.dst_id == NODE_ID) {
      Serial. printf("[DATA] Received from node %d:  %s\n", pkt.src_id, pkt. payload);
    } else {
      // Forward data packet
      RouteEntry* route = findRoute(pkt.dst_id);
      if (route) {
        pkt.last_hop = NODE_ID;
        pkt. hop_count++;
        sendPkt(route->next_ip, pkt);
        Serial.printf("[DATA] Forwarded from %d to %d via %d\n", pkt.src_id, pkt. dst_id, route->next_hop);
      } else {
        Serial.printf("[DATA] No route to node %d, dropping packet\n", pkt.dst_id);
      }
    }
  }
}

/* ================= RREQ INITIATION ================= */
void sendRREQ(uint8_t dst) {
  MeshPacket rreq{};
  rreq.type = MSG_RREQ;
  rreq.src_id = NODE_ID;
  rreq. dst_id = dst;
  rreq.last_hop = NODE_ID;
  rreq.hop_count = 0;
  rreq.seq = ++seqCounter;
  snprintf(rreq.payload, sizeof(rreq.payload), "RREQ from node %d", NODE_ID);

  sendPkt(BROADCAST_IP, rreq);
  Serial.printf("[RREQ] Initiated route discovery to node %d (seq: %d)\n", dst, rreq.seq);
}

/* ================= DATA SEND ================= */
void sendData(uint8_t dst, const char* message) {
  RouteEntry* route = findRoute(dst);
  if (!route) {
    Serial.printf("[DATA] No route to node %d, initiating RREQ\n", dst);
    sendRREQ(dst);
    return;
  }

  MeshPacket data{};
  data.type = MSG_DATA;
  data.src_id = NODE_ID;
  data.dst_id = dst;
  data.last_hop = NODE_ID;
  data.hop_count = 0;
  data.seq = ++seqCounter;
  strncpy(data. payload, message, sizeof(data.payload) - 1);
  data.payload[sizeof(data.payload) - 1] = '\0';

  sendPkt(route->next_ip, data);
  Serial.printf("[DATA] Sent to node %d via %d: %s\n", dst, route->next_hop, message);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.printf("\n[INIT] Starting AODV Node %d\n", NODE_ID);

  WiFi.mode(WIFI_STA);
  WiFi. begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WIFI] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("[WIFI] Connected!  IP: %s\n", WiFi. localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
}

/* ================= LOOP ================= */
void loop() {
  // Check for incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize == sizeof(MeshPacket)) {
    MeshPacket pkt;
    udp. read((uint8_t*)&pkt, sizeof(pkt));
    IPAddress senderIP = udp.remoteIP();
    handlePacket(pkt, senderIP);
  }

  // Example: Node 1 sends data to Node 5 after 10 seconds
  if (NODE_ID == 1 && ! dataSent && millis() > 10000) {
    sendData(5, "Hello from Node 1!");
    dataSent = true;
  }

  delay(10);
}
