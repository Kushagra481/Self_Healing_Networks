/************************************************************
 * AODV Routing Protocol over ESP-NOW (Arduino ESP32)
 * Each ESP32 acts as a routing node
 ************************************************************/

#include <Arduino.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
}

/* ================= CONFIG ================= */
#define MAC_LEN 6
#define MAX_ROUTES 10
#define MAX_RREQ_CACHE 20
#define RREQ_CACHE_TIMEOUT_MS 5000
#define WIFI_CHANNEL 1

#define MSG_RREQ  1
#define MSG_RREP  2
#define MSG_DATA  3
#define MSG_RERR  4
#define MSG_HELLO 5

#define HELLO_INTERVAL_MS 15000
#define DATA_INTERVAL_MS  5000

/* ================= STRUCTS ================= */

typedef struct {
  uint8_t type;
  uint8_t src[MAC_LEN];
  uint8_t dst[MAC_LEN];
  uint16_t seq;
  uint16_t rreq_id;
  uint8_t hops;
  char payload[100];
} message_t;

typedef struct {
  uint8_t dest[MAC_LEN];
  uint8_t next_hop[MAC_LEN];
  uint8_t hops;
  uint16_t seq;
  bool valid;
} route_t;

typedef struct {
  uint8_t src[MAC_LEN];
  uint16_t rreq_id;
  int64_t ts;
} rreq_cache_t;

/* ================= GLOBALS ================= */

uint8_t BROADCAST_MAC[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

route_t routes[MAX_ROUTES];
int route_count = 0;

rreq_cache_t rreq_cache[MAX_RREQ_CACHE];
int rreq_count = 0;

SemaphoreHandle_t route_mutex;

uint16_t local_seq = 0;
uint16_t rreq_counter = 0;

/* CHANGE THIS PER DESTINATION NODE */
uint8_t TARGET_NODE[6] = {0x30,0xC9,0x22,0x12,0xEC,0xC4};

/* ================= UTILS ================= */

void get_self_mac(uint8_t *mac) {
  esp_wifi_get_mac(WIFI_IF_STA, mac);
}

bool mac_eq(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, MAC_LEN) == 0;
}

bool is_self(const uint8_t *mac) {
  uint8_t self[6];
  get_self_mac(self);
  return mac_eq(mac, self);
}

int64_t now_ms() {
  return esp_timer_get_time() / 1000;
}

void print_mac(const uint8_t *m) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                m[0], m[1], m[2], m[3], m[4], m[5]);
}

/* ================= ROUTING ================= */

int find_route(const uint8_t *dest) {
  for (int i = 0; i < route_count; i++)
    if (mac_eq(routes[i].dest, dest) && routes[i].valid)
      return i;
  return -1;
}

void update_route(const uint8_t *dest,
                  const uint8_t *next,
                  uint8_t hops,
                  uint16_t seq) {

  int i;
  for (i = 0; i < route_count; i++)
    if (mac_eq(routes[i].dest, dest)) break;

  if (i == route_count && route_count < MAX_ROUTES) {
    memcpy(routes[i].dest, dest, MAC_LEN);
    route_count++;
  }

  if (i < MAX_ROUTES) {
    memcpy(routes[i].next_hop, next, MAC_LEN);
    routes[i].hops = hops;
    routes[i].seq = seq;
    routes[i].valid = true;
  }
}

/* ================= RREQ CACHE ================= */

bool rreq_seen(const uint8_t *src, uint16_t id) {
  int64_t t = now_ms();
  for (int i = 0; i < rreq_count; i++) {
    if (mac_eq(rreq_cache[i].src, src) &&
        rreq_cache[i].rreq_id == id &&
        t - rreq_cache[i].ts < RREQ_CACHE_TIMEOUT_MS)
      return true;
  }
  return false;
}

void cache_rreq(const uint8_t *src, uint16_t id) {
  if (rreq_count < MAX_RREQ_CACHE) {
    memcpy(rreq_cache[rreq_count].src, src, MAC_LEN);
    rreq_cache[rreq_count].rreq_id = id;
    rreq_cache[rreq_count].ts = now_ms();
    rreq_count++;
  }
}

/* ================= ESP-NOW ================= */

void ensure_peer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, MAC_LEN);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);
}

void on_recv(const esp_now_recv_info_t *info,
             const uint8_t *data,
             int len) {

  if (len != sizeof(message_t)) return;

  message_t msg;
  memcpy(&msg, data, sizeof(msg));

  xSemaphoreTake(route_mutex, portMAX_DELAY);

  if (msg.type == MSG_RREQ) {
    if (!rreq_seen(msg.src, msg.rreq_id)) {
      cache_rreq(msg.src, msg.rreq_id);
      update_route(msg.src, info->src_addr, msg.hops + 1, msg.seq);

      if (is_self(msg.dst)) {
        message_t rrep{};
        rrep.type = MSG_RREP;
        memcpy(rrep.src, msg.dst, MAC_LEN);
        memcpy(rrep.dst, msg.src, MAC_LEN);
        rrep.seq = ++local_seq;
        ensure_peer(info->src_addr);
        esp_now_send(info->src_addr, (uint8_t *)&rrep, sizeof(rrep));
      } else {
        msg.hops++;
        esp_now_send(BROADCAST_MAC, (uint8_t *)&msg, sizeof(msg));
      }
    }
  }

  else if (msg.type == MSG_RREP) {
    update_route(msg.src, info->src_addr, msg.hops + 1, msg.seq);
  }

  else if (msg.type == MSG_DATA) {
    if (is_self(msg.dst)) {
      Serial.print("DATA RECEIVED: ");
      Serial.println(msg.payload);
    } else {
      int r = find_route(msg.dst);
      if (r >= 0) {
        ensure_peer(routes[r].next_hop);
        esp_now_send(routes[r].next_hop,
                      (uint8_t *)&msg,
                      sizeof(msg));
      }
    }
  }

  xSemaphoreGive(route_mutex);
}

/* ================= TASKS ================= */

void hello_task(void *) {
  while (true) {
    message_t m{};
    m.type = MSG_HELLO;
    get_self_mac(m.src);
    esp_now_send(BROADCAST_MAC, (uint8_t *)&m, sizeof(m));
    vTaskDelay(pdMS_TO_TICKS(HELLO_INTERVAL_MS));
  }
}

void data_task(void *) {
  while (true) {
    message_t d{};
    d.type = MSG_DATA;
    get_self_mac(d.src);
    memcpy(d.dst, TARGET_NODE, MAC_LEN);
    strcpy(d.payload, "GPS:12.9716,77.5946");

    int r = find_route(TARGET_NODE);
    if (r >= 0) {
      ensure_peer(routes[r].next_hop);
      esp_now_send(routes[r].next_hop,
                    (uint8_t *)&d,
                    sizeof(d));
    } else {
      message_t rreq{};
      rreq.type = MSG_RREQ;
      get_self_mac(rreq.src);
      memcpy(rreq.dst, TARGET_NODE, MAC_LEN);
      rreq.rreq_id = rreq_counter++;
      esp_now_send(BROADCAST_MAC, (uint8_t *)&rreq, sizeof(rreq));
    }

    vTaskDelay(pdMS_TO_TICKS(DATA_INTERVAL_MS));
  }
}

/* ================= INIT ================= */

void app_main() {
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_register_recv_cb(on_recv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, MAC_LEN);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  route_mutex = xSemaphoreCreateMutex();

  xTaskCreate(hello_task, "HELLO", 4096, NULL, 1, NULL);
  xTaskCreate(data_task, "DATA", 4096, NULL, 1, NULL);

  Serial.println("AODV NODE STARTED");
}

/* ================= ARDUINO BRIDGE ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);
  static bool started = false;
  if (!started) {
    started = true;
    app_main();
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
