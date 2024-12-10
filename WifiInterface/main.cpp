#define CYW43_SPI_PIO_PREFERRED_PIO 1
#include <math.h>
#include <stdio.h>

#include "camera.h"
#include "ak8419.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico_ws_server/web_socket_server.h"
#include "tiff.h"

extern "C" {
#include "include/dhcpserver.h"
#include "include/dnsserver.h"
}
#define MAX_CON 5
uint32_t linecount = 0;
uint32_t conn_list[MAX_CON];
extern int8_t camera_state;

void on_connect(WebSocketServer &server, uint32_t conn_id) {
  printf("WebSocket opened conn_id:%d\n", conn_id);
  server.sendMessage(conn_id, "hello");
  bool connected = false;
  for (int i = 0; i < MAX_CON; i++) {
    if (conn_list[i] == 0) {
      conn_list[i] = conn_id;
      connected = true;
      break;
    }
  }
}

void on_disconnect(WebSocketServer &server, uint32_t conn_id) {
  printf("WebSocket closed\n");
  for (int i = 0; i < MAX_CON; i++) {
    if (conn_list[i] == conn_id) {
      conn_list[i] = 0;
    }
  }
}

void on_message(WebSocketServer &server, uint32_t conn_id, const void *data,
                size_t len) {
  printf("WebSocket message with length %d\n", (int)len);
  for (int i = 0; i < (int)len; i++) {
    printf("%d:", ((uint8_t *)data)[i]);
  }
  printf("%d", *((uint16_t *)data));
  printf("\n");
  if ((int)len == 18 && *((uint16_t *)data) == 1337) {
    // command recieved
    struct web_command test_command;
    memcpy(&test_command, (uint8_t *)data + 2, 16);

    multicore_fifo_push_blocking((uint32_t)&test_command);
    switch (test_command.command) {
    case COMMAND_CAPTURE:
      printf("Capture started");
      break;
    case COMMAND_FOCUS:
      printf("focusing");
      break;
    case COMMAND_PREVIEW:
      printf("previewing");
      break;
    case COMMAND_ABORT:
      printf("Aborting");
      break;
    case COMMAND_EXPOSE:
      printf("Exposure calculation");
      break;
    default:
      break;
    }
    printf("exposure time:%d\n", test_command.exp_time);
    printf("lines:%d\n", test_command.lines);
  }
}

int main() {
set_sys_clock_khz(CLOCK_SPEED/1000,true);
  stdio_init_all();
  sleep_ms(1000);
  if (sd_init()) {
    printf("SD card mounted");
  }
  if (cyw43_arch_init() != 0) {
    printf("cyw43_arch_init failed\n");
    while (1)
      tight_loop_contents();
  }
  const char *ap_name = "picow_test";
#if 1
  const char *password = "password";
#else
  const char *password = NULL;
#endif

  cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

  ip_addr_t mask;
  ip_addr_t gateway;
  IP4_ADDR(ip_2_ip4(&gateway), 192, 168, 4, 1);
  IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

  // Start the dhcp server
  dhcp_server_t dhcp_server;
  dhcp_server_init(&dhcp_server, &gateway, &mask);

  // Start the dns server
  dns_server_t dns_server;
  dns_server_init(&dns_server, &gateway);

  printf("Connected.\n");
  for (int i = 0; i < MAX_CON; i++) {
    conn_list[i] = 0;
  }

  WebSocketServer server;
  server.setConnectCallback(on_connect);
  server.setCloseCallback(on_disconnect);
  server.setMessageCallback(on_message);
  printf("Starting server at %s on port %u\n",
         ip4addr_ntoa(netif_ip4_addr(netif_list)), 80);

  bool server_ok = server.startListening(80);
  if (!server_ok) {
    printf("Failed to start WebSocket server\n");
    while (1) {
      tight_loop_contents();
    }
  }
  multicore_launch_core1(camera_task);

  printf("WebSocket server started\n");
  uint32_t timedelta = time_us_32();
  while (1) {
    if (multicore_fifo_rvalid()) {
      uint32_t msg = multicore_fifo_pop_blocking();
      uint16_t *msg_len = (uint16_t *)((uint8_t *)msg + 2);
      printf("msg_len: %i\n", *msg_len);
      for (int i = 0; i < MAX_CON; i++) {
        if (conn_list[i]) {
          server.sendMessage(conn_list[i], (uint8_t *)msg, *msg_len);
        }
      }
      timedelta = time_us_32();
      printf("sending line\n");
    }
    if (camera_state != COMMAND_CAPTURE) {
      cyw43_arch_poll();
    }
  }
  dns_server_deinit(&dns_server);
  dhcp_server_deinit(&dhcp_server);
  cyw43_arch_deinit();
}
