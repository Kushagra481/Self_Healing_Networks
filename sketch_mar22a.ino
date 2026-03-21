#include <WiFi.h>
#include <WiFiUdp.h>

/* ================= CONFIG ================= */
#define WIFI_SSID     "YOUR_WIFI_NAME"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD" 
#define UDP_PORT      5005
#define BROADCAST_IP  IPAddress(255,255,255,255)

#define NODE_ID       1     // CHANGE PER ESP32 (1-5)
#define MAX_HOPS      10
#define MAX_ROUTES    20
#define MAX_NEIGHBORS 10
#define MAX_CACHE     30

/* ================= TIMEOUTS ================= */
#define HELLO_INTERVAL      3000   // Send hello every 3s
#define NEIGHBOR_TIMEOUT    10000  // Neighbor dead after 10s
#define ROUTE_TIMEOUT       20000  // Route expires after 20s
#define RERR_RATELIMIT      2000   // Min 2s between RERR for same dest
#define RREQ_RETRY_INTERVAL 5000   // Retry RREQ after 5s
#define MAX_RETRIES         3      // Max retransmission attempts

/* ================= MESSAGE TYPES ================= */
#define MSG_RREQ    1
#define MSG_RREP    2
#define MSG_DATA    3
#define MSG_RERR    4
#define MSG_HELLO   5

/* ================= OPTIONAL FEATURES ================= */
#define ENABLE_ENCRYPTION   false  // Set true to enable (requires crypto library)
#define ENABLE_METRICS      true   // Track RSSI, delay, etc.
#define PRINT_ROUTING_TABLE true   // Print table periodically
#define TABLE_PRINT_INTERVAL 10000 // Print every 10s

WiFiUDP udp;

/* ================= STRUCTURES ================= */
struct MeshPacket {
  uint8_t type;
  uint8_t src_id;
  uint8_t dst_id;
  uint8_t last_hop;
  uint8_t hop_count;
  uint8_t ttl;
  uint32_t seq;
  uint32_t timestamp;     // For delay measurement
  char payload[64];
  uint8_t checksum;       // Simple integrity check
};

struct RouteEntry {
  uint8_t dst_id;
  uint8_t next_hop;
  IPAddress next_ip;
  uint8_t hop_count;
  uint32_t seq;
  unsigned long expiry;
  unsigned long last_used;
  bool active;
  int8_t rssi;           // Signal strength
  uint16_t delay_ms;     // Average delay
  uint8_t loss_rate;     // Packet loss percentage
};

struct Neighbor {
  uint8_t id;
  IPAddress ip;
  unsigned long lastSeen;
  int8_t rssi;
  uint8_t hello_count;   // Count received hellos
  bool alive;
};

struct CacheEntry {
  uint8_t src;
  uint32_t seq;
  unsigned long ts;
};

struct PendingPacket {
  MeshPacket pkt;
  uint8_t retries;
  unsigned long last_attempt;
  bool active;
};

struct RERREntry {
  uint8_t dst_id;
  unsigned long last_sent;
};

/* ================= GLOBALS ================= */
RouteEntry routes[MAX_ROUTES];
Neighbor neighbors[MAX_NEIGHBORS];
CacheEntry cache[MAX_CACHE];
PendingPacket pending[10];
RERREntry rerr_sent[10];

int routeCount = 0;
int neighborCount = 0;
int cacheCount = 0;
int pendingCount = 0;
int rerrCount = 0;

uint32_t seqCounter = 0;
unsigned long lastHello = 0;
unsigned long lastTablePrint = 0;
bool dataSent = false;

/* ================= STATISTICS ================= */
struct Stats {
  uint32_t packets_sent;
  uint32_t packets_received;
  uint32_t packets_forwarded;
  uint32_t packets_dropped;
  uint32_t rreq_sent;
  uint32_t rrep_sent;
  uint32_t rerr_sent;
  uint32_t retransmissions;
} stats = {0};

/* ================= UTILITY FUNCTIONS ================= */

uint8_t calculateChecksum(MeshPacket &pkt) {
  uint8_t sum = 0;
  sum ^= pkt.type;
  sum ^= pkt.src_id;
  sum ^= pkt.dst_id;
  sum ^= pkt.hop_count;
  sum ^= (pkt.seq & 0xFF);
  sum ^= ((pkt.seq >> 8) & 0xFF);
  return sum;
}

bool verifyChecksum(MeshPacket &pkt) {
  uint8_t calculated = calculateChecksum(pkt);
  return (calculated == pkt.checksum);
}

RouteEntry* findRoute(uint8_t dst) {
  unsigned long now = millis();
  for (int i = 0; i < routeCount; i++) {
    if (routes[i].dst_id == dst && routes[i].active) {
      // Check if route expired
      if (now > routes[i].expiry) {
        routes[i].active = false;
        Serial.printf("[ROUTE EXPIRED] Dst=%d\n", dst);
        continue;
      }
      return &routes[i];
    }
  }
  return nullptr;
}

void addRoute(uint8_t dst, uint8_t nextHop, IPAddress ip, uint8_t hops, uint32_t seq, int8_t rssi) {
  unsigned long now = millis();
  
  // Update existing route
  for (int i = 0; i < routeCount; i++) {
    if (routes[i].dst_id == dst) {
      // Only update if sequence number is newer or same seq with better hop count
      if (seq > routes[i].seq || (seq == routes[i].seq && hops < routes[i].hop_count)) {
        routes[i].next_hop = nextHop;
        routes[i].next_ip = ip;
        routes[i].hop_count = hops;
        routes[i].seq = seq;
        routes[i].expiry = now + ROUTE_TIMEOUT;
        routes[i].last_used = now;
        routes[i].active = true;
        routes[i].rssi = rssi;
        Serial.printf("[ROUTE UPDATED] Dst=%d NextHop=%d Hops=%d Seq=%u RSSI=%d\n", 
                      dst, nextHop, hops, seq, rssi);
      } else {
        // Just refresh timeout
        routes[i].expiry = now + ROUTE_TIMEOUT;
      }
      return;
    }
  }
  
  // Add new route
  if (routeCount < MAX_ROUTES) {
    routes[routeCount++] = {
      dst, nextHop, ip, hops, seq, 
      now + ROUTE_TIMEOUT, now, true, rssi, 0, 0
    };
    Serial.printf("[ROUTE ADDED] Dst=%d NextHop=%d Hops=%d Seq=%u\n", dst, nextHop, hops, seq);
  } else {
    Serial.println("[ERROR] Routing table full!");
  }
}

void invalidateRoute(uint8_t dst) {
  for (int i = 0; i < routeCount; i++) {
    if (routes[i].dst_id == dst) {
      routes[i].active = false;
      Serial.printf("[ROUTE INVALIDATED] Dst=%d\n", dst);
    }
  }
}

bool seenPacket(uint8_t src, uint32_t seq) {
  unsigned long now = millis();
  
  // Cleanup old cache entries
  for (int i = 0; i < cacheCount; i++) {
    if (now - cache[i].ts > 10000) {
      cache[i] = cache[--cacheCount];
      i--;
    }
  }
  
  // Check if seen
  for (int i = 0; i < cacheCount; i++) {
    if (cache[i].src == src && cache[i].seq == seq) {
      return true;
    }
  }
  
  // Add to cache
  if (cacheCount < MAX_CACHE) {
    cache[cacheCount++] = {src, seq, now};
  }
  
  return false;
}

Neighbor* findNeighbor(uint8_t id) {
  for (int i = 0; i < neighborCount; i++) {
    if (neighbors[i].id == id) {
      return &neighbors[i];
    }
  }
  return nullptr;
}

void addNeighbor(uint8_t id, IPAddress ip, int8_t rssi) {
  unsigned long now = millis();
  
  Neighbor* n = findNeighbor(id);
  if (n) {
    n->lastSeen = now;
    n->rssi = rssi;
    n->hello_count++;
    n->alive = true;
  } else {
    if (neighborCount < MAX_NEIGHBORS) {
      neighbors[neighborCount++] = {id, ip, now, rssi, 1, true};
      Serial.printf("[NEIGHBOR ADDED] ID=%d IP=%s RSSI=%d\n", 
                    id, ip.toString().c_str(), rssi);
    }
  }
}

void checkNeighborTimeout() {
  unsigned long now = millis();
  
  for (int i = 0; i < neighborCount; i++) {
    if (now - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT) {
      if (neighbors[i].alive) {
        Serial.printf("[NEIGHBOR LOST] ID=%d\n", neighbors[i].id);
        neighbors[i].alive = false;
        
        // Invalidate routes through this neighbor
        for (int j = 0; j < routeCount; j++) {
          if (routes[j].next_hop == neighbors[i].id && routes[j].active) {
            invalidateRoute(routes[j].dst_id);
            sendRERR(routes[j].dst_id);
          }
        }
      }
    }
  }
}

bool shouldSendRERR(uint8_t dst) {
  unsigned long now = millis();
  
  // Rate limit RERR messages
  for (int i = 0; i < rerrCount; i++) {
    if (rerr_sent[i].dst_id == dst) {
      if (now - rerr_sent[i].last_sent < RERR_RATELIMIT) {
        return false;  // Too soon
      }
      rerr_sent[i].last_sent = now;
      return true;
    }
  }
  
  // First RERR for this dest
  if (rerrCount < 10) {
    rerr_sent[rerrCount++] = {dst, now};
  }
  return true;
}

void addPendingPacket(MeshPacket &pkt) {
  for (int i = 0; i < 10; i++) {
    if (!pending[i].active) {
      pending[i].pkt = pkt;
      pending[i].retries = 0;
      pending[i].last_attempt = millis();
      pending[i].active = true;
      pendingCount++;
      return;
    }
  }
}

void processPending() {
  unsigned long now = millis();
  
  for (int i = 0; i < 10; i++) {
    if (!pending[i].active) continue;
    
    if (now - pending[i].last_attempt < RREQ_RETRY_INTERVAL) continue;
    
    // Check if route now exists
    RouteEntry* route = findRoute(pending[i].pkt.dst_id);
    if (route) {
      // Route found! Send packet
      sendPkt(route->next_ip, pending[i].pkt);
      pending[i].active = false;
      pendingCount--;
      Serial.printf("[PENDING RESOLVED] Dst=%d\n", pending[i].pkt.dst_id);
    } else {
      // Retry or give up
      if (pending[i].retries < MAX_RETRIES) {
        sendRREQ(pending[i].pkt.dst_id);
        pending[i].retries++;
        pending[i].last_attempt = now;
        stats.retransmissions++;
        Serial.printf("[PENDING RETRY] Dst=%d Attempt=%d\n", 
                      pending[i].pkt.dst_id, pending[i].retries + 1);
      } else {
        Serial.printf("[PENDING FAILED] Dst=%d - Dropping\n", pending[i].pkt.dst_id);
        pending[i].active = false;
        pendingCount--;
        stats.packets_dropped++;
      }
    }
  }
}

void sendPkt(IPAddress ip, MeshPacket &pkt) {
  pkt.checksum = calculateChecksum(pkt);
  pkt.timestamp = millis();
  
  udp.beginPacket(ip, UDP_PORT);
  udp.write((uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
  
  stats.packets_sent++;
}

/* ================= HELLO PACKETS ================= */
void sendHello() {
  MeshPacket pkt{};
  pkt.type = MSG_HELLO;
  pkt.src_id = NODE_ID;
  pkt.ttl = 1;  // Only 1 hop
  pkt.seq = ++seqCounter;
  
  sendPkt(BROADCAST_IP, pkt);
}

/* ================= RERR (Route Error) ================= */
void sendRERR(uint8_t dst) {
  if (!shouldSendRERR(dst)) return;
  
  MeshPacket pkt{};
  pkt.type = MSG_RERR;
  pkt.src_id = NODE_ID;
  pkt.dst_id = dst;
  pkt.seq = ++seqCounter;
  pkt.ttl = MAX_HOPS;
  
  sendPkt(BROADCAST_IP, pkt);
  stats.rerr_sent++;
  
  Serial.printf("[RERR] Sent for dst=%d\n", dst);
}

/* ================= RREQ ================= */
void sendRREQ(uint8_t dst) {
  MeshPacket pkt{};
  pkt.type = MSG_RREQ;
  pkt.src_id = NODE_ID;
  pkt.dst_id = dst;
  pkt.last_hop = NODE_ID;
  pkt.hop_count = 0;
  pkt.seq = ++seqCounter;
  pkt.ttl = MAX_HOPS;
  
  sendPkt(BROADCAST_IP, pkt);
  stats.rreq_sent++;
  
  Serial.printf("[RREQ] Initiated for dst=%d seq=%u\n", dst, pkt.seq);
}

/* ================= PACKET HANDLING ================= */
void handlePacket(MeshPacket &pkt, IPAddress senderIP) {
  // Basic validation
  if (pkt.ttl == 0) {
    stats.packets_dropped++;
    return;
  }
  
  if (pkt.src_id == NODE_ID) return;  // Ignore own packets
  
  if (!verifyChecksum(pkt)) {
    Serial.println("[ERROR] Checksum failed!");
    stats.packets_dropped++;
    return;
  }
  
  pkt.ttl--;
  stats.packets_received++;
  
  // Get RSSI if available
  int8_t rssi = WiFi.RSSI();
  
  // Duplicate detection (except HELLO)
  if (pkt.type != MSG_HELLO) {
    if (seenPacket(pkt.src_id, pkt.seq)) {
      return;  // Already processed
    }
  }
  
  // Update neighbor (all packet types update neighbor info)
  if (pkt.type == MSG_HELLO || pkt.last_hop == pkt.src_id) {
    addNeighbor(pkt.src_id, senderIP, rssi);
  }
  
  // Process by type
  switch (pkt.type) {
    case MSG_HELLO:
      // Already handled by addNeighbor above
      break;
      
    case MSG_RREQ:
      handleRREQ(pkt, senderIP, rssi);
      break;
      
    case MSG_RREP:
      handleRREP(pkt, senderIP, rssi);
      break;
      
    case MSG_DATA:
      handleDATA(pkt, senderIP);
      break;
      
    case MSG_RERR:
      handleRERR(pkt);
      break;
  }
}

void handleRREQ(MeshPacket &pkt, IPAddress senderIP, int8_t rssi) {
  // Add reverse route to source
  addRoute(pkt.src_id, pkt.last_hop, senderIP, pkt.hop_count + 1, pkt.seq, rssi);
  
  if (pkt.dst_id == NODE_ID) {
    // We are destination - send RREP
    MeshPacket rrep{};
    rrep.type = MSG_RREP;
    rrep.src_id = NODE_ID;
    rrep.dst_id = pkt.src_id;
    rrep.last_hop = NODE_ID;
    rrep.hop_count = 0;
    rrep.seq = ++seqCounter;
    rrep.ttl = MAX_HOPS;
    snprintf(rrep.payload, sizeof(rrep.payload), "Route to %d", NODE_ID);
    
    RouteEntry* route = findRoute(pkt.src_id);
    if (route) {
      sendPkt(route->next_ip, rrep);
      stats.rrep_sent++;
      Serial.printf("[RREP] Sent to %d via %d\n", pkt.src_id, route->next_hop);
    }
  } else {
    // Forward RREQ
    pkt.last_hop = NODE_ID;
    pkt.hop_count++;
    sendPkt(BROADCAST_IP, pkt);
    stats.packets_forwarded++;
    Serial.printf("[RREQ] Forwarded: src=%d dst=%d hops=%d\n", 
                  pkt.src_id, pkt.dst_id, pkt.hop_count);
  }
}

void handleRREP(MeshPacket &pkt, IPAddress senderIP, int8_t rssi) {
  // Add forward route to destination
  addRoute(pkt.src_id, pkt.last_hop, senderIP, pkt.hop_count + 1, pkt.seq, rssi);
  
  if (pkt.dst_id == NODE_ID) {
    // RREP for us - route established
    uint16_t delay = millis() - pkt.timestamp;
    RouteEntry* route = findRoute(pkt.src_id);
    if (route && ENABLE_METRICS) {
      route->delay_ms = delay;
    }
    Serial.printf("[RREP] Route to %d established (hops=%d, delay=%ums)\n", 
                  pkt.src_id, pkt.hop_count + 1, delay);
  } else {
    // Forward RREP toward destination
    RouteEntry* route = findRoute(pkt.dst_id);
    if (route) {
      pkt.last_hop = NODE_ID;
      pkt.hop_count++;
      sendPkt(route->next_ip, pkt);
      stats.packets_forwarded++;
      Serial.printf("[RREP] Forwarded to %d via %d\n", pkt.dst_id, route->next_hop);
    }
  }
}

void handleDATA(MeshPacket &pkt, IPAddress senderIP) {
  if (pkt.dst_id == NODE_ID) {
    // Data for us
    uint16_t delay = millis() - pkt.timestamp;
    Serial.printf("[DATA] From %d: %s (delay=%ums)\n", pkt.src_id, pkt.payload, delay);
  } else {
    // Forward data
    RouteEntry* route = findRoute(pkt.dst_id);
    if (route) {
      route->last_used = millis();
      pkt.last_hop = NODE_ID;
      pkt.hop_count++;
      sendPkt(route->next_ip, pkt);
      stats.packets_forwarded++;
      Serial.printf("[DATA] Forwarded: src=%d dst=%d via=%d\n", 
                    pkt.src_id, pkt.dst_id, route->next_hop);
    } else {
      Serial.printf("[DATA] No route to %d, dropping\n", pkt.dst_id);
      stats.packets_dropped++;
      sendRERR(pkt.dst_id);
    }
  }
}

void handleRERR(MeshPacket &pkt) {
  Serial.printf("[RERR] Received for dst=%d\n", pkt.dst_id);
  
  // Invalidate route
  invalidateRoute(pkt.dst_id);
  
  // Propagate RERR if we had active route
  for (int i = 0; i < routeCount; i++) {
    if (routes[i].dst_id == pkt.dst_id && !routes[i].active) {
      sendRERR(pkt.dst_id);
      break;
    }
  }
}

/* ================= DATA SEND ================= */
void sendData(uint8_t dst, const char* message) {
  RouteEntry* route = findRoute(dst);
  
  if (!route) {
    Serial.printf("[DATA] No route to %d, initiating RREQ\n", dst);
    
    // Create packet and add to pending
    MeshPacket data{};
    data.type = MSG_DATA;
    data.src_id = NODE_ID;
    data.dst_id = dst;
    data.last_hop = NODE_ID;
    data.hop_count = 0;
    data.seq = ++seqCounter;
    data.ttl = MAX_HOPS;
    strncpy(data.payload, message, sizeof(data.payload) - 1);
    
    addPendingPacket(data);
    sendRREQ(dst);
    return;
  }
  
  // Send data
  MeshPacket data{};
  data.type = MSG_DATA;
  data.src_id = NODE_ID;
  data.dst_id = dst;
  data.last_hop = NODE_ID;
  data.hop_count = 0;
  data.seq = ++seqCounter;
  data.ttl = MAX_HOPS;
  strncpy(data.payload, message, sizeof(data.payload) - 1);
  data.payload[sizeof(data.payload) - 1] = '\0';
  
  sendPkt(route->next_ip, data);
  route->last_used = millis();
  
  Serial.printf("[DATA] Sent to %d via %d: %s\n", dst, route->next_hop, message);
}

/* ================= ROUTING TABLE DISPLAY ================= */
void printRoutingTable() {
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.printf("║  ROUTING TABLE - Node %d                                   ║\n", NODE_ID);
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  Serial.println("║ Dst | Next | IP              | Hops | Seq  | RSSI | Delay ║");
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  
  if (routeCount == 0) {
    Serial.println("║                      (No routes)                           ║");
  } else {
    for (int i = 0; i < routeCount; i++) {
      if (!routes[i].active) continue;
      
      char line[70];
      snprintf(line, sizeof(line), 
               "║  %d  |  %d   | %-15s |  %d   | %4u | %4d | %4ums ║",
               routes[i].dst_id,
               routes[i].next_hop,
               routes[i].next_ip.toString().c_str(),
               routes[i].hop_count,
               routes[i].seq,
               routes[i].rssi,
               routes[i].delay_ms);
      Serial.println(line);
    }
  }
  
  Serial.println("╚════════════════════════════════════════════════════════════╝");
  
  // Neighbor table
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║  NEIGHBOR TABLE                                            ║");
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  Serial.println("║ ID  | IP              | RSSI | Hellos | Status             ║");
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  
  if (neighborCount == 0) {
    Serial.println("║                    (No neighbors)                          ║");
  } else {
    for (int i = 0; i < neighborCount; i++) {
      char line[70];
      snprintf(line, sizeof(line),
               "║  %d  | %-15s | %4d |   %3d  | %-18s ║",
               neighbors[i].id,
               neighbors[i].ip.toString().c_str(),
               neighbors[i].rssi,
               neighbors[i].hello_count,
               neighbors[i].alive ? "ALIVE" : "DEAD");
      Serial.println(line);
    }
  }
  
  Serial.println("╚════════════════════════════════════════════════════════════╝");
  
  // Statistics
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║  STATISTICS                                                ║");
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  Serial.printf("║  Packets Sent:       %6u                                 ║\n", stats.packets_sent);
  Serial.printf("║  Packets Received:   %6u                                 ║\n", stats.packets_received);
  Serial.printf("║  Packets Forwarded:  %6u                                 ║\n", stats.packets_forwarded);
  Serial.printf("║  Packets Dropped:    %6u                                 ║\n", stats.packets_dropped);
  Serial.printf("║  RREQ Sent:          %6u                                 ║\n", stats.rreq_sent);
  Serial.printf("║  RREP Sent:          %6u                                 ║\n", stats.rrep_sent);
  Serial.printf("║  RERR Sent:          %6u                                 ║\n", stats.rerr_sent);
  Serial.printf("║  Retransmissions:    %6u                                 ║\n", stats.retransmissions);
  Serial.printf("║  Pending Packets:    %6d                                 ║\n", pendingCount);
  Serial.println("╚════════════════════════════════════════════════════════════╝\n");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.printf("║  ENHANCED AODV MESH NETWORK - Node %d                      ║\n", NODE_ID);
  Serial.println("╚════════════════════════════════════════════════════════════╝\n");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WiFi] Signal: %d dBm\n\n", WiFi.RSSI());
  
  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on port %d\n\n", UDP_PORT);
  
  Serial.println("Features enabled:");
  Serial.println("  ✓ Route expiration (TTL)");
  Serial.println("  ✓ Route error (RERR)");
  Serial.println("  ✓ Node failure detection");
  Serial.println("  ✓ Retransmission on failure");
  Serial.println("  ✓ Hello packets");
  Serial.println("  ✓ Route refresh");
  Serial.println("  ✓ Checksum validation");
  Serial.printf("  %s Metrics (RSSI, delay)\n", ENABLE_METRICS ? "✓" : "✗");
  Serial.printf("  %s Encryption\n\n", ENABLE_ENCRYPTION ? "✓" : "✗");
}

/* ================= MAIN LOOP ================= */
void loop() {
  unsigned long now = millis();
  
  // Send HELLO packets
  if (now - lastHello > HELLO_INTERVAL) {
    sendHello();
    lastHello = now;
  }
  
  // Check for dead neighbors
  checkNeighborTimeout();
  
  // Process pending packets (retransmission)
  processPending();
  
  // Print routing table periodically
  if (PRINT_ROUTING_TABLE && now - lastTablePrint > TABLE_PRINT_INTERVAL) {
    printRoutingTable();
    lastTablePrint = now;
  }
  
  // Handle incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize == sizeof(MeshPacket)) {
    MeshPacket pkt;
    udp.read((uint8_t*)&pkt, sizeof(pkt));
    IPAddress senderIP = udp.remoteIP();
    handlePacket(pkt, senderIP);
  }
  
static unsigned long lastSend = 0;

if (NODE_ID == 1 && now > 15000) {

  if (now - lastSend > 15000) {  // every 15 seconds

    sendData(2, "Hello Node 2");
    sendData(3, "Hello Node 3");
    sendData(4, "Hello Node 4");
    sendData(5, "Hello Node 5");

    lastSend = now;
  }

}
  
  delay(10);
}


