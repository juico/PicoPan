#define TCP_TMR_INTERVAL       20
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "pico/cyw43_arch.h"

#include "camera.h"
#include "ak8419.h"
#include "pico/util/queue.h"

#include "pico/multicore.h"
#include "pico_ws_server/web_socket_server.h"
#include "tiff.h"
#include "hardware/clocks.h"
extern "C" {
#include "include/dhcpserver.h"
#include "include/dnsserver.h"
}
#include <lwip/stats.h>

#include "SdFat.h"
extern SdExFat sd;

#include <vector>
#include <string>
#include <cstring>
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define MAX_CON 5
#define CHUNK_SIZE 8192
volatile uint32_t linecount = 0;
void sdio_log(const char *txt, uint32_t arg1, uint32_t arg2){
    printf(txt);
    printf(" %i , %i\n ",arg1,arg2);
}
 bool sd_write_active=0;
uint32_t conn_list[MAX_CON];
static uint8_t io_buffer[CHUNK_SIZE];
extern int8_t camera_state;
queue_t commandqueue;
queue_t dataqueue;
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
    queue_add_blocking(&commandqueue,&test_command);
    //multicore_fifo_push_blocking((uint32_t)&test_command);
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
  } else {
        std::vector<char> msgBuffer(len + 1);
        memcpy(msgBuffer.data(), data, len);
        msgBuffer[len] = '\0';
        std::string msg(msgBuffer.data());
        if (msg.rfind("LIST", 0) == 0) {
            // Komenda listowania plików
            ExFile dir;
            std::string listStr = "LIST:";
            if (dir.openRoot(&sd)) {
                ExFile file;
                while (file.openNext(&dir, O_RDONLY)) {
                    if (!file.isDir()) {
                        char name[64];
                        file.getName(name, sizeof(name));
                        uint64_t size = file.fileSize();
                        listStr += name;
                        listStr += ";";
                        listStr += std::to_string(size);
                        listStr += "\n";
                    }
                    file.close();
                }
                dir.close();
            }
            server.sendMessage(conn_id, listStr.c_str());  // wysłanie listy jako tekst
        } else if (msg.rfind("DEL:", 0) == 0) {
            // Komenda usunięcia pliku
            std::string filename = msg.substr(4);
            ExFile root;
            bool success = false;
            if (root.openRoot(&sd)) {
                std::string beginDel = "DELETING:" + filename;
                server.sendMessage(conn_id, beginDel.c_str());
                if (root.remove(filename.c_str())) {
                    success = true;
                }
                root.close();
            }
            if (success) {
                std::string okMsg = "DELETED:" + filename;
                server.sendMessage(conn_id, okMsg.c_str());
            } else {
                std::string err = "ERROR: Could not delete file " + filename;
                server.sendMessage(conn_id, err.c_str());
            }
        }
    }
}

static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    if (!p) { 
        // Connection closed by client
        tcp_close(pcb);
        return ERR_OK;
    }
    tcp_recved(pcb, p->tot_len);  // free TCP window for received data

    char *req = (char*)p->payload;
    if (strncmp(req, "GET /file/", 10) == 0) {
        // Handle file download request
        char *spc = strchr(req + 10, ' ');
        if (spc) {
            *spc = '\0';  // terminate the filename string
            std::string fname(req + 10);
            ExFile f;
            if (f.open(fname.c_str(), O_RDONLY)) {
                // File found – send HTTP 200 header with Content-Length
                char hdr[256];
                sprintf(hdr,
                        "HTTP/1.1 200 OK\r\n"
                        "Content-Type: application/octet-stream\r\n"
                        "Content-Length: %llu\r\n"
                        "Content-Disposition: attachment; filename=\"%s\"\r\n"
                        "Connection: close\r\n\r\n",
                        (unsigned long long)f.fileSize(), fname.c_str());
                tcp_write(pcb, hdr, strlen(hdr), TCP_WRITE_FLAG_COPY);

                // Send file data in chunks
                int n;
                while ((n = f.read(io_buffer, CHUNK_SIZE)) > 0) {
                    int off = 0;
                    while (off < n) {
                        err_t e = tcp_write(pcb, io_buffer + off, n - off, TCP_WRITE_FLAG_COPY);
                        if (e == ERR_MEM) {
                            // Not enough buffer space – flush and retry
                            tcp_output(pcb);
                            cyw43_arch_poll();
                            continue;
                        }
                        if (e != ERR_OK) {
                            // Some other error occurred – abort sending
                            f.close();
                            tcp_output(pcb);
                            tcp_close(pcb);
                            pbuf_free(p);
                            return ERR_OK;
                        }
                        off += (n - off);  // all bytes of this chunk queued
                    }
                    tcp_output(pcb);       // push out the data
                    cyw43_arch_poll();     // poll WiFi driver
                }
                f.close();
                // Flush any remaining data and close connection
                tcp_output(pcb);
                tcp_close(pcb);
                pbuf_free(p);
                return ERR_OK;
            }
            // File not found – send 404 and close
            const char *notFound = 
                "HTTP/1.1 404 Not Found\r\n"
                "Connection: close\r\n"
                "\r\n";
            tcp_write(pcb, notFound, strlen(notFound), TCP_WRITE_FLAG_COPY);
            tcp_output(pcb);
            tcp_close(pcb);
            pbuf_free(p);
            return ERR_OK;
        }
    }
    // Bad request – send 400 and close
    const char *badReq = 
        "HTTP/1.1 400 Bad Request\r\n"
        "Connection: close\r\n"
        "\r\n";
    tcp_write(pcb, badReq, strlen(badReq), TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
    tcp_close(pcb);
    pbuf_free(p);
    return ERR_OK;
}
static err_t http_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    // Set up callbacks for the new connection
    tcp_recv(newpcb, http_recv);
    // We do not set tcp_sent here – connection will be closed manually in http_recv
    return ERR_OK;
}

static void start_http() {
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    tcp_bind(pcb, IP_ANY_TYPE, 80);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, http_accept);
}

int main() {
set_sys_clock_khz(CLOCK_SPEED/1000,true);
queue_init(&commandqueue,sizeof(web_command), 4);
queue_init(&dataqueue,sizeof(web_data),4);
  stdio_init_all();
  //sleep_ms(10000);
    if (sd_init()) {
    printf("SD card mounted");
  }
    multicore_launch_core1(camera_task);
//sleep_ms(1000);
    if (cyw43_arch_init() != 0) {
    printf("cyw43_arch_init failed\n");
    while (1)
      tight_loop_contents();
  }


  const char *ap_name = "PicoPan";
#if 1
  const char *password = "password";
#else
  const char *password = NULL;
#endif

  cyw43_arch_enable_ap_mode(ap_name, 0, CYW43_AUTH_OPEN);
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
         ip4addr_ntoa(netif_ip4_addr(netif_list)), 81);

  bool server_ok = server.startListening(81);
  if (!server_ok) {
    printf("Failed to start WebSocket server\n");
    while (1) {
      tight_loop_contents();
    }
  }

  printf("WebSocket server started\n");
  start_http();
  uint32_t timedelta = time_us_32();
  while (1) {
      if (!sd_write_active && !queue_is_empty(&dataqueue)) {
          struct web_data new_data;
          if (queue_try_remove(&dataqueue, &new_data)) {
              for (int i = 0; i < MAX_CON; i++) {
                if (!conn_list[i]) continue;

                bool is_ascii = (new_data.buffer[0] >= ' ' && new_data.buffer[0] <= '~');

                if (is_ascii) {
                    new_data.buffer[new_data.length] = '\0';
                    server.sendMessage(conn_list[i], (const char*)new_data.buffer);
                } else {
                    server.sendMessage(conn_list[i], new_data.buffer, new_data.length);
                }
            }
          }
      }
      if (!sd_write_active) {
          cyw43_arch_poll();
          server.popMessages();
      }
  }
  dns_server_deinit(&dns_server);
  dhcp_server_deinit(&dhcp_server);
  cyw43_arch_deinit();
}
