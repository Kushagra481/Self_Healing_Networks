#include <WiFi.h>
#include <WiFiUdp.h>

/* ================= CONFIG ================= */
#define WIFI_SSID     "OnePlus Nord CE5 p2jr"
#define WIFI_PASS     "vvci6957"
#define UDP_PORT      5005
#define BROADCAST_IP  IPAddress(255,255,255,255)

#define NODE_ID       5     // CHANGE PER ESP32 (1..5)
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
    if (rreqCache[i].src == src && rreqCache[i].seq == seq)
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
    if (dataCache[i].src == src && dataCache[i].seq == seq)
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

    addRoute(pkt.src_id, pkt.last_hop, senderIP, pkt.hop_count + 1, pkt.seq);

    if (pkt.dst_id == NODE_ID) {
      MeshPacket rrep{};
      rrep.type = MSG_RREP;
      rrep.src_id = NODE_ID;
      rrep.dst
