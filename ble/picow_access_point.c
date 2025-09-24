/**
 * @file picow_access_point.c
 * @brief Wi-Fi Access Point server for Raspberry Pi Pico W that collects and serves CAN bus data
 * 
 * This program creates a Wi-Fi access point and implements a simple HTTP server
 * that collects CAN bus data using the MCP2515 controller and J1939 protocol.
 * The collected data is stored in a buffer and can be retrieved via HTTP endpoints.
 * 
 * Main features:
 * - Wi-Fi Access Point with DHCP and DNS server
 * - HTTP server with endpoints for data retrieval
 * - CAN bus data collection using MCP2515 and J1939 protocol
 * - Automatic buffer management for collected data
 * - Interrupt-based WiFi connection instead of polling
 */

 #include <stdio.h>
 #include <string.h>
 #include <ctype.h>
 #include <stdlib.h>
 #include "btstack.h"
 #include "btstack_config.h"
 #include "hardware/uart.h"
 #include "hardware/gpio.h"
 #include "pico/stdlib.h"
 #include "pico/cyw43_arch.h"
 #include "uart.h"
 #include "lwip/pbuf.h"
 #include "lwip/tcp.h"
 #include "dhcpserver.h"
 #include "dnsserver.h"
 #include "j1939.h"
 #include "mcp2515.h"
 #include "ble/att_server.h"
 #include "hardware/sync.h"
#include "btstack_run_loop_embedded.h" 



#define UART_ID uart1
#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define BAUD_RATE 115200

#define MOTOR_PIN 0
#define DEVICE_NAME "FleetDevice"
#define EXPECTED_KEY "123"
#define MAX_KEY_LENGTH 16
#define HEARTBEAT_INTERVAL 1000
#define KEY_INTERVAL 10000
#define FIRMWARE_CMD "Firmware Update"

volatile bool ble_done = false;
static bool led_state = false;

static char rx_key[16] = {0};
static bool ble_connected = false;
static bool notifications_enabled = false;
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static btstack_timer_source_t heartbeat_timer, validate_key;
static uint8_t data[31];
static uint8_t current_key[MAX_KEY_LENGTH];
static uint16_t current_key_length = 0;
bool ble_authenticated = false;
static bool switch_requested = false;
static char switch_command[32] = {0};

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_context_callback_registration_t tx_sender_callback;

static uint8_t adv_data[] = {
        0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
        0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1A, 0x18
    };

#define BLE_BUFFER_SIZE 252
char ble_data_buffer[BLE_BUFFER_SIZE];

static absolute_time_t last_can_activity_time;
#define CAN_INACTIVITY_TIMEOUT_US 30000000

//  /* Buffer configuration for CAN data collection */
//  #define MAX_CAN_BATCH_RESPONSE 10280     
//  #define BUFFER_AUTO_CLEAR_INTERVAL_US 30000000  // 30 seconds in microseconds
 
 #define TCP_PORT 80                      // HTTP server port
 #define DEBUG_printf printf              // Debug print function alias
 #define POLL_TIME_S 5                    // Poll timeout in seconds
 #define HTTP_GET "GET"                   // HTTP GET method string
 #define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/plain; charset=utf-8\nConnection: close\n\n"
 #define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s/\n\n"
 
 /* Buffer configuration for CAN data collection */
 #define MAX_CAN_BATCH_RESPONSE 8192     // Maximum size of CAN data buffer (64KB), but i reduce it b'cause giving out of memory issue
 #define BUFFER_AUTO_CLEAR_INTERVAL_US 30000000  // 30 seconds in microseconds
 
 /* Interrupt event flags */
 #define EVENT_CAN_DATA_READY  0x01
 #define EVENT_CLIENT_ACTIVE   0x02
 #define EVENT_SLEEP_TIMEOUT   0x04
 #define PICO_DEFAULT_LED_PIN 0  // GPIO 0 is the onboard LED for Pico W

 bool isSwitched = false;
 bool client_connected = false; // Tracks Wi-Fi client
 struct tcp_pcb *connected_client_pcb = NULL;

 /**
  * @brief TCP Server state structure
  */
 typedef struct TCP_SERVER_T_ {
     struct tcp_pcb *server_pcb;          // TCP protocol control block for the server
     bool complete;                       // Flag to indicate server completion
     ip_addr_t gw;                        // Gateway IP address
 } TCP_SERVER_T;
 
 /**
  * @brief TCP Connection state structure for client connections
  */
 #define MAX_HTTP_RESULT_LEN 4096
 
 typedef struct TCP_CONNECT_STATE_T_ {
     struct tcp_pcb *pcb;
     int sent_len;
     char headers[128];
     char result[MAX_HTTP_RESULT_LEN]; // Increased from 1024
     int header_len;
     int result_len;
     ip_addr_t *gw;
 } TCP_CONNECT_STATE_T;

 typedef struct {
     bool client_connected;        // Flag indicating if any client is connected
     uint8_t connected_clients;    // Count of connected clients
     absolute_time_t last_activity; // Time of last client activity
     bool sleep_mode_active;       // Flag indicating if sleep mode is active
 } POWER_MANAGEMENT_T;

 static POWER_MANAGEMENT_T power_state = {
     .client_connected = false,
     .connected_clients = 0,
     .sleep_mode_active = false
 };

 typedef enum {
    MODE_BLE,
    MODE_WIFI_AP
 } OperationMode;

 static OperationMode current_mode = MODE_BLE;

 static char can_batch_response[MAX_CAN_BATCH_RESPONSE];  // Buffer for collected CAN data
 static int can_batch_offset = 0;                         // Current position in the buffer
 static absolute_time_t last_clear_time;                  // Time when buffer was last cleared
 static bool buffer_full_reported = false;                // Flag to prevent repeated buffer full messages
 static bool new_data_collected = false;  
 static int ble_data_offset = 0;

 bool start = false;   // Flag to avoid sleeping in start

bool init_can_system();
void enter_sleep_mode();
void exit_sleep_mode();
bool power_management_callback(struct repeating_timer *timer);
bool can_check_callback(struct repeating_timer *timer);
void update_current_key(const uint8_t* buffer, uint16_t size);
void handle_disconnection();
void handle_att_can_send_now_event();
void ble_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
int att_write_callback(hci_con_handle_t conn_handle, uint16_t att_handle, uint16_t mode,
                      uint16_t offset, uint8_t *buffer, uint16_t size);
static void key_handler(btstack_timer_source_t *ts);
void clear_can_batch_buffer();
const char* get_all_can_batch_data();
static int hex_to_int(char c);
static void url_decode(char *dst, const char *src);
static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err);
static void tcp_server_close(TCP_SERVER_T *state);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len);
static int test_server_content(const char *request, const char *params, char *result, size_t max_result_len);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb);
static void tcp_server_err(void *arg, err_t err);
static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err);
static bool tcp_server_open(void *arg, const char *ap_name);
// void gpio_callback(uint gpio, uint32_t event_mask);
// void key_pressed_func(void *param);
void switch_mode();
void ble_mode();
void wifi_ap_mode();

bool init_can_system() {
    printf("Initializing mcp2515...\n");
    if (!mcp2515_init()) {
        printf("MCP2515 initialization failed\n");
        return false;
    }
    
    printf("CAN system initialized successfully\n");
    return true;
}

 static struct repeating_timer power_management_timer;
 static struct repeating_timer can_check_timer;
 
 /**
  * @brief Enters low-power mode to save energy
  * 
  * This function configures the Pico W for minimal power consumption
  * while maintaining the ability to wake up when a client connects.
  */
void enter_sleep_mode() {
    if (power_state.sleep_mode_active) return;
    printf("Entering sleep mode...\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    
    if (current_mode == MODE_WIFI_AP) {
        cyw43_arch_lwip_begin();
        cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
        cyw43_arch_lwip_end();
    } else {
        hci_power_control(HCI_POWER_SLEEP);
    }
    
    power_state.sleep_mode_active = true;
}

/**
 * @brief Exits low-power mode when client activity is detected
 * 
 * This function restores normal operation mode when clients connect.
 */
void exit_sleep_mode() {
    if (!power_state.sleep_mode_active) return;
    printf("Exiting sleep mode\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    
    if (current_mode == MODE_WIFI_AP) {
        cyw43_arch_lwip_begin();
        cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
        cyw43_arch_lwip_end();
    } else {
        hci_power_control(HCI_POWER_ON);
        gap_advertisements_enable(1);
    }
    
    power_state.sleep_mode_active = false;
}

void handle_disconnection() {
    printf("Handling disconnection - resetting all states\n");

    ble_connected = false;
    con_handle = HCI_CON_HANDLE_INVALID;
    ble_authenticated = false;
    notifications_enabled = false;
    power_state.client_connected = false;
    
    memset(current_key, 0, MAX_KEY_LENGTH);
    current_key_length = 0;

    memset(ble_data_buffer, 0, BLE_BUFFER_SIZE);

    can_batch_offset = 0;
    ble_data_offset = 0;
    memset(can_batch_response, 0, sizeof(can_batch_response));
    last_clear_time = get_absolute_time();
    buffer_full_reported = false;
    new_data_collected = false;

    power_state.last_activity = get_absolute_time();

    if(switch_command == "switchToBLE"){
        uint8_t name_len = strlen(DEVICE_NAME);
        data[0] = name_len + 1;
        data[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
        memcpy(&data[2], DEVICE_NAME, name_len);

        gap_advertisements_set_params(0x0020, 0x0020, 0, 0, NULL, 0x07, 0x00);
        gap_advertisements_set_data(sizeof(adv_data), (uint8_t *)adv_data);
        gap_scan_response_set_data(name_len + 2, data);
        gap_advertisements_enable(1);
        
        printf("Advertising restarted\n");
    }

    led_state = false;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

    btstack_run_loop_remove_timer(&validate_key);

    if (power_state.sleep_mode_active) {
        power_state.sleep_mode_active = false;
        hci_power_control(HCI_POWER_ON);
        gap_advertisements_enable(1);
        printf("Exited sleep mode for reconnection\n");
    }
}

void switch_mode() {
    printf("Compare value: %d",strcmp(switch_command, "Firmware Update"));
    printf("Switching mode: %s\n", switch_command);
    
    if (strcmp(switch_command, "Firmware Update") == 0) {
        // Clean up BLE
        printf("Ello");
        // handle_disconnection();
        // gap_disconnect(con_handle);
        // hci_power_control(HCI_POWER_OFF);
        btstack_run_loop_trigger_exit();
        // Switch to Wi-Fi AP
        current_mode = MODE_WIFI_AP;
        memset(switch_command, 0, sizeof(switch_command));
        // wifi_ap_mode();

    } 
    else if (strcmp(switch_command, "switchToBLE") == 0) {
        // Clean up Wi-Fi
        cyw43_arch_disable_ap_mode();
        cyw43_arch_deinit();
        
        // Switch to BLE
        current_mode = MODE_BLE;
        memset(switch_command, 0, sizeof(switch_command));
        switch_requested = false;

    }
}

 void clear_can_batch_buffer() {
     
     can_batch_offset = 0;
     memset(can_batch_response, 0, sizeof(can_batch_response));
     last_clear_time = get_absolute_time();
     buffer_full_reported = false;
     
     printf("Buffer cleared.\n");
 }

  /**
  * @brief Returns the current CAN data buffer content
  * 
  * @return The CAN data buffer content or a message if empty
  */
 const char* get_all_can_batch_data() {
     return (can_batch_offset == 0) ? "No CAN data received yet" : can_batch_response;
 }
 

/**
 * @brief Timer callback for power management
 * 
 * Checks if the device should enter sleep mode due to inactivity
 */
int in_power = 0;
bool power_management_callback(struct repeating_timer *timer) {
    absolute_time_t current_time = get_absolute_time();

    if(ble_done) return false;
    

    if(switch_requested){
        in_power++;
        printf("%d\n",in_power);
        printf("I got u");
        switch_mode();
        return false;
    }

    if (!power_state.client_connected) {
        static int heartbeat_counter = 0;
        if (++heartbeat_counter % 5 == 0) {  
            printf("."); 
        }
    }

    bool should_sleep = !power_state.sleep_mode_active && 
                       !power_state.client_connected && start &&
                       !ble_connected && 
                       absolute_time_diff_us(power_state.last_activity, current_time) > 5000000; 

    if (should_sleep) {
        printf("Entering sleep mode - no client connection and no CAN activity\n");
        enter_sleep_mode();
    }

    if (power_state.client_connected && ble_connected) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    } else if (ble_connected) {
        static bool blink_state = false;
        blink_state = !blink_state;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, blink_state);
    } else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); 
    }
    
    return true; 
}

void update_current_key(const uint8_t* buffer, uint16_t size){
    current_key_length = (size < MAX_KEY_LENGTH - 1) ? size : MAX_KEY_LENGTH - 1;
    
    memset(current_key, 0, MAX_KEY_LENGTH);
    
    memcpy(current_key, buffer, current_key_length);
    
    current_key[current_key_length] = '\0';

    uart_puts(UART_ID, "Key updated: ");
    uart_puts(UART_ID, (char*)current_key);
    uart_puts(UART_ID, "\n");
    printf("Key Updated: %s (length: %d)\n", current_key, current_key_length);
}

bool can_data_received_this_cycle = false;
bool can_check_callback(struct repeating_timer *timer) {
    // Only process CAN data when clients are connected
    if (!power_state.client_connected) {
        return true; // Keep the timer running
    }
    
    absolute_time_t current_time = get_absolute_time(); 
    can_data_received_this_cycle = false;
    
    // Collect as many CAN messages as possible in a burst
    int message_count = 0;
    int max_messages_per_cycle = 130;

    while (message_count < max_messages_per_cycle) {
        CANMessage msg;
        
        if (mcp2515_check_receive()) {
            if (j1939_receive_message(&msg)) {
                char* formatted_data = format_can_data_for_app(&msg);

                int data_len = strlen(formatted_data);
                if (can_batch_offset + data_len + 2 < MAX_CAN_BATCH_RESPONSE) {
                    strcpy(&can_batch_response[can_batch_offset], formatted_data);
                    can_batch_offset += data_len;
                    can_batch_response[can_batch_offset++] = '\n';  
                    can_batch_response[can_batch_offset] = '\0';   
                    // Set flags for data availability
                    new_data_collected = true;
                    can_data_received_this_cycle = true;

                    last_can_activity_time = current_time;
                    
                    // Update power management
                    power_state.last_activity = current_time;
                } else if (!buffer_full_reported) {
                    printf("Buffer full! Can't append more data. Clearing the CAN buffer...\n");
                    can_batch_offset = 0;
                    ble_data_offset = 0;
                    memset(can_batch_response, 0, sizeof(can_batch_response));
                    last_clear_time = get_absolute_time();
                    // last_can_activity_time = get_absolute_time();
                    buffer_full_reported = false;
                    new_data_collected = false;
                }
                message_count++;
            }
        } else {
            break; 
        }
    } 

    if (can_data_received_this_cycle) {
        printf("Collected %d CAN messages\n", message_count);
       
        if (power_state.sleep_mode_active) {
            exit_sleep_mode();
        }
    }    

    if (new_data_collected && notifications_enabled && ble_authenticated && 
        con_handle != HCI_CON_HANDLE_INVALID && can_batch_offset > ble_data_offset) {
        att_server_request_can_send_now_event(con_handle);
    }

    return true;
}

int att_write_callback(hci_con_handle_t conn_handle, uint16_t att_handle, uint16_t mode,
                       uint16_t offset, uint8_t *buffer, uint16_t size) {
    // For log
    // printf("=== ATT Write Callback ===\n");
    // printf("Connection handle: 0x%04X, ATT handle: 0x%04X, Size: %d\n", 
    //        conn_handle, att_handle, size);

    power_state.last_activity = get_absolute_time();

    // Handle notification enable/disable
    if (att_handle == ATT_CHARACTERISTIC_94f3c8a5_a161_468f_9844_de92dd90ace7_01_CLIENT_CONFIGURATION_HANDLE) {
        bool new_notifications_enabled = little_endian_read_16(buffer, 0) ==
                                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        
        printf("Notifications: %d -> %d\n", notifications_enabled, new_notifications_enabled);

        if (new_notifications_enabled && !notifications_enabled && ble_authenticated) {
            notifications_enabled = true;
            con_handle = conn_handle;
            ble_data_offset = 0; 
            
            printf("Notifications enabled - checking for immediate data\n");
            if (can_batch_offset > 0) {
                printf("Sending immediate CAN data: %d bytes\n", can_batch_offset);
                att_server_request_can_send_now_event(con_handle);
            }
        } else {
            notifications_enabled = new_notifications_enabled;
            if (new_notifications_enabled) {
                con_handle = conn_handle;
            }
        }
        return 0;
    }

    // Handle data writes (authentication and commands)
    if (att_handle == ATT_CHARACTERISTIC_94f3c8a4_a161_468f_9844_de92dd90ace7_01_VALUE_HANDLE) {
        char msg[64] = {0};
        int copy_len = (size < 63) ? size : 63;
        memcpy(msg, buffer, copy_len);
        msg[copy_len] = '\0';

        if (!ble_authenticated) {
            // Handle authentication
            if (strncmp(msg, EXPECTED_KEY, strlen(EXPECTED_KEY)) == 0) {
                ble_authenticated = true;
                power_state.client_connected = true;
                ble_data_offset = 0; 
                
                printf("BLE Authenticated with key: %s\n", msg);
                memcpy(current_key, msg, copy_len);
                current_key_length = copy_len;
                att_server_notify(con_handle, ATT_CHARACTERISTIC_94f3c8a5_a161_468f_9844_de92dd90ace7_01_VALUE_HANDLE, current_key, current_key_length);
                exit_sleep_mode();

                if (notifications_enabled && can_batch_offset > 0) {
                    printf("Authentication complete - sending available data\n");
                    att_server_request_can_send_now_event(con_handle);
                }
            }
            else {
                printf("Invalid BLE key: %s\n", msg);
            }
            update_current_key(buffer, size);
        } else {
            printf("BLE Data received: %s\n", msg);
            if(strncmp(msg, FIRMWARE_CMD, strlen(FIRMWARE_CMD)) == 0){
                printf("Switching to Wifi access point mode !!!!");
                strncpy(switch_command, "Firmware Update", sizeof(switch_command));
                switch_requested = true;
                return 0;
            }

            uart_write_blocking(UART_ID, buffer, size);

            if (notifications_enabled && can_batch_offset > ble_data_offset) {
                printf("Command received - sending available CAN data\n");
                att_server_request_can_send_now_event(con_handle);
            }
        }
    }   
    return 0;
}

static void key_handler(btstack_timer_source_t *ts){
    printf("Validating key...\n");
    printf("Expected key: '%s' (length: %d)\n", EXPECTED_KEY, (int)strlen(EXPECTED_KEY));
    printf("Current key: '%s' (length: %d)\n", current_key, current_key_length);
    
    if(current_key_length == 0) {
        uart_puts(UART_ID, "No key received yet, disconnecting...\n");
        printf("No key received yet, disconnecting...\n");
        if(con_handle != HCI_CON_HANDLE_INVALID){
            gap_disconnect(con_handle);
        }
        return;
    }
 
    if(strcmp((char*)current_key, EXPECTED_KEY) != 0){
        uart_puts(UART_ID, "Invalid key, Disconnecting...\n");
        printf("Invalid key, Disconnecting...\n");
        if(con_handle != HCI_CON_HANDLE_INVALID){
            gap_disconnect(con_handle);
        } else {
            uart_puts(UART_ID, "No active connection\n");
            printf("No active connection\n");
        }
        led_state = false;  
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
    } else {
        uart_puts(UART_ID, "Key verified successfully\n");
        printf("Authentication successful - clearing CAN buffer for fresh data\n");
        start = true;
        can_batch_offset = 0;
        memset(can_batch_response, 0, sizeof(can_batch_response));
        last_clear_time = get_absolute_time();
        buffer_full_reported = false; 

        led_state = true;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        power_state.client_connected = true;
        exit_sleep_mode();
    }
}

void handle_att_can_send_now_event() {
    static absolute_time_t last_notify_time;
    if (!notifications_enabled || !ble_authenticated || con_handle == HCI_CON_HANDLE_INVALID) {
        printf("Cannot send: notifications=%d, auth=%d, handle=0x%04X\n", 
               notifications_enabled, ble_authenticated, con_handle);
        return;
    }

    if(ble_data_offset >= BLE_BUFFER_SIZE) printf("BLE buffer full....");

    int remaining = can_batch_offset - ble_data_offset;
    if (remaining <= 0) {
        printf("No data to send (offset=%d, total=%d)\n", ble_data_offset, can_batch_offset);
        // Reset for next batch
        can_batch_offset = 0;
        ble_data_offset = 0;
        memset(can_batch_response, 0, sizeof(can_batch_response));
        last_clear_time = get_absolute_time();
        buffer_full_reported = false;
        return;
    }

    int chunk_size = (remaining < BLE_BUFFER_SIZE - 1) ? remaining : BLE_BUFFER_SIZE - 1;
    printf("Chunk size %d", chunk_size);
    memcpy(ble_data_buffer, &can_batch_response[ble_data_offset], chunk_size);
    ble_data_buffer[chunk_size] = '\0';
    
    // printf("Sending chunk: %d bytes (offset=%d, remaining=%d)\n", 
    //        chunk_size, ble_data_offset, remaining);

    //timer 
    absolute_time_t current_time = get_absolute_time();
    int64_t elapsed_us = absolute_time_diff_us(last_notify_time, current_time);
    // printf(" Notification sent after: %lld us (%0.2f ms)\n", elapsed_us, elapsed_us / 1000.0);
    last_notify_time = current_time;

    int result = att_server_notify(con_handle,
        ATT_CHARACTERISTIC_94f3c8a5_a161_468f_9844_de92dd90ace7_01_VALUE_HANDLE,
        (uint8_t *)ble_data_buffer,
        chunk_size);

    if (result == 0) {
        ble_data_offset += chunk_size;

        if (ble_data_offset < can_batch_offset) {

            att_server_request_can_send_now_event(con_handle);
        } else {
            printf("All data transmitted - resetting buffers\n");
            can_batch_offset = 0;
            ble_data_offset = 0;
            memset(can_batch_response, 0, sizeof(can_batch_response));
            // memset(ble_data_buffer, 0, sizeof(ble_data_buffer));
            last_clear_time = get_absolute_time();
            buffer_full_reported = false;
        }
    } else {
        printf("Failed to send notification (result=%d)\n", result);
    }
}

  /**
  * @brief Updates the client connection state
  * 
  * @param connected True if a client has connected, false if disconnected
  */
 static void update_connection_state(bool connected) {
     cyw43_arch_lwip_begin();
     
     if (connected) {
         power_state.connected_clients++;
         if (!power_state.client_connected) {
             power_state.client_connected = true;
             exit_sleep_mode();
         }
     } else {
         if (power_state.connected_clients > 0) {
             power_state.connected_clients--;
         }
         
         if (power_state.connected_clients == 0) {
             power_state.client_connected = false;
             // Don't enter sleep immediately - timer will handle this
         }
     }
     
     power_state.last_activity = get_absolute_time();
     cyw43_arch_lwip_end();
 }

 /**
  * @brief Closes a client TCP connection with interrupt-safe handling
  * 
  * @param con_state Connection state structure
  * @param client_pcb Client TCP protocol control block
  * @param close_err Error code to return
  * @return Error code
  */
 static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err) {
     if (client_pcb) {
         assert(con_state && con_state->pcb == client_pcb);
         
         // Update connection state - client disconnected
         update_connection_state(false);
         
         // Remove all callbacks
         tcp_arg(client_pcb, NULL);
         tcp_poll(client_pcb, NULL, 0);
         tcp_sent(client_pcb, NULL);
         tcp_recv(client_pcb, NULL);
         tcp_err(client_pcb, NULL);
         
         // Close the connection
         err_t err = tcp_close(client_pcb);
         if (err != ERR_OK) {
             DEBUG_printf("close failed %d, calling abort\n", err);
             tcp_abort(client_pcb);
             close_err = ERR_ABRT;
         }
         
         // Free the connection state
         if (con_state) {
             free(con_state);
         }
     }
     return close_err;
 }
 
 /**
  * @brief Closes the TCP server
  * 
  * @param state Server state structure
  */
 static void tcp_server_close(TCP_SERVER_T *state) {
     if (state->server_pcb) {
         tcp_arg(state->server_pcb, NULL);
         tcp_close(state->server_pcb);
         state->server_pcb = NULL;
     }
 }
 
 /**
  * @brief TCP sent callback function
  * 
  * Called when data has been sent from the server to the client.
  * 
  * @param arg Connection state structure
  * @param pcb TCP protocol control block
  * @param len Number of bytes sent
  * @return Error code
  */
 static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
     TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
     DEBUG_printf("tcp_server_sent %u\n", len);
 
     con_state->sent_len += len;
 
     if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
         DEBUG_printf("All data sent. Clearing buffer.\n");
         clear_can_batch_buffer();
         return tcp_close_client_connection(con_state, pcb, ERR_OK);
     }
 
     return ERR_OK;
 }
 
 /**
  * @brief Processes HTTP requests and generates responses
  * 
  * @param request HTTP request path
  * @param params HTTP query parameters
  * @param result Buffer for response content
  * @param max_result_len Maximum length of response
  * @return Length of response content
  */
 static int test_server_content(const char *request, const char *params, char *result, size_t max_result_len) {
     int len = 0;
 
     if (strcmp(request, "/") == 0) {
         len = snprintf(result, max_result_len,
             "KION CAN Data Server\n"
             "Available endpoints:\n"
             "/getLatestData - Get and clear the latest CAN data\n"
             "/getCurrentData - Get current CAN data without clearing\n"
             "/status - Server status information\n"
             "Buffer size: %d bytes", can_batch_offset);
     } else if (strncmp(request, "/getLatestData", 14) == 0) {
        
         const char* latest_data = get_all_can_batch_data();
         len = snprintf(result, max_result_len, "%s", latest_data);
         
        //  printf("Sending %d bytes of CAN data via /getLatestData\n", len);
         // Do not clear here, will do after full send
     } else if (strncmp(request, "/getCurrentData", 15) == 0) {
        
         const char* current_data = get_all_can_batch_data();
         len = snprintf(result, max_result_len, "%s", current_data);
        
         printf("Sending %d bytes of CAN data via /getCurrentData (buffer not cleared)\n", len);
     } else if (strncmp(request, "/status", 7) == 0) {
         len = snprintf(result, max_result_len,
             "Server Status:\n"
             "Buffer utilization: %d/%d bytes (%.1f%%)\n"
             "Last buffer clear: %llu ms ago\n"
             "Clients connected: %d\n"
             "Sleep mode: %s\n",
             can_batch_offset, MAX_CAN_BATCH_RESPONSE,
             (float)can_batch_offset / MAX_CAN_BATCH_RESPONSE * 100.0f,
             absolute_time_diff_us(last_clear_time, get_absolute_time()) / 1000,
             power_state.connected_clients,
             power_state.sleep_mode_active ? "Active" : "Inactive");
     } else {
         len = snprintf(result, max_result_len, "Unknown endpoint. Try /getLatestData or /getCurrentData");
     }
 
     return len;
 }
 
 static char *strcasestr_local(const char *haystack, const char *needle) {

    size_t needle_len = strlen(needle);

    if (needle_len == 0) return (char *)haystack;
 
    for (; *haystack; ++haystack) {

        if (strncasecmp(haystack, needle, needle_len) == 0) {

            return (char *)haystack;

        }

    }

    return NULL;

}
 /**
  * @brief TCP receive callback function
  * 
  * Called when data is received from a client.
  * 
  * @param arg Connection state structure
  * @param pcb TCP protocol control block
  * @param p Packet buffer containing received data
  * @param err Error code
  * @return Error code
  */
// static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
//     TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T *)arg;

//     static bool header_parsed = false;
//     static int content_length = 0;
//     static int total_received = 0;
//     static char header_buffer[4096];
//     static int header_len = 0;

//     if (!p) {
//         DEBUG_printf("Connection closed\n");
//         header_parsed = false;
//         header_len = 0;
//         content_length = 0;
//         total_received = 0;
//         return tcp_close_client_connection(con_state, pcb, ERR_OK);
//     }

//     assert(con_state && con_state->pcb == pcb);
//     tcp_recved(pcb, p->tot_len);

//     // Debug: print raw incoming data
//     struct pbuf *q = p;
//     DEBUG_printf("\n=== Incoming data (p->tot_len = %d) ===\n", p->tot_len);
//     while (q) {
//         fwrite(q->payload, 1, q->len, stdout);
//         q = q->next;
//     }

//     if (!header_parsed) {
//         // Accumulate header
//         size_t to_copy = p->tot_len < sizeof(header_buffer) - header_len - 1 ?
//                          p->tot_len : sizeof(header_buffer) - header_len - 1;

//         pbuf_copy_partial(p, header_buffer + header_len, to_copy, 0);
//         header_len += to_copy;
//         header_buffer[header_len] = '\0';

//         // Handle GET first (no body)
//         if (strncmp(header_buffer, "GET ", 4) == 0) {
//             DEBUG_printf("Received GET request\n");

//             const char *response_body = "Hello from PicoW!";
//             char response[512];
//             int response_len = snprintf(response, sizeof(response),
//                 "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: %d\r\n\r\n%s",
//                 (int)strlen(response_body), response_body);

//             tcp_write(pcb, response, response_len, TCP_WRITE_FLAG_COPY);
//             tcp_output(pcb);
//             pbuf_free(p);
//             return tcp_close_client_connection(con_state, pcb, ERR_OK);
//         }

//         // Handle POST
//         char *body_start = strstr(header_buffer, "\r\n\r\n");
//         if (body_start) {
//             header_parsed = true;
//             body_start += 4;
//             int header_size = body_start - header_buffer;
//             int body_in_header = header_len - header_size;

//             // Content-Length
//             char *cl = strcasestr_local(header_buffer, "Content-Length:");
//             if (cl) {
//                 cl += 15;
//                 while (*cl == ' ') cl++;
//                 content_length = atoi(cl);
//                 DEBUG_printf("Parsed Content-Length: %d\n", content_length);
//             }

//             // Optional File-Name
//             // char *fname = strcasestr_local(header_buffer, "File-Name:");
//             // char filename[64] = "firmware.hex";
//             // if (fname) {
//             //     fname += 10;
//             //     while (*fname == ' ') fname++;
//             //     char *end = strchr(fname, '\r');
//             //     if (end) *end = '\0';
//             //     strncpy(filename, fname, sizeof(filename) - 1);
//             //     filename[sizeof(filename) - 1] = '\0';
//             // }

//             // Process body in header
//             DEBUG_printf("Received initial body (%d bytes)\n", body_in_header);
//             fwrite(body_start, 1, body_in_header, stdout);
//             total_received = body_in_header;

//             // Here you could parse and forward this data to CAN

//             pbuf_free(p);
//             return ERR_OK;
//         }

//         // Header too large
//         if (header_len >= sizeof(header_buffer) - 1) {
//             DEBUG_printf("Header too large! Aborting.\n");
//             const char *resp = "HTTP/1.1 431 Header Too Large\r\n\r\n";
//             tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//             tcp_output(pcb);
//             pbuf_free(p);
//             return tcp_close_client_connection(con_state, pcb, ERR_OK);
//         }

//         pbuf_free(p);
//         return ERR_OK;
//     }

//     // === Continue receiving body ===
//     q = p;
//     while (q) {
//         fwrite(q->payload, 1, q->len, stdout);  // print or forward to CAN
//         total_received += q->len;
//         q = q->next;
//     }

//     pbuf_free(p);

//     if (total_received >= content_length) {
//         DEBUG_printf("Upload complete! Total %d bytes\n", total_received);

//         const char *resp = "HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n";
//         tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//         tcp_output(pcb);

//         // Reset state
//         header_parsed = false;
//         header_len = 0;
//         content_length = 0;
//         total_received = 0;

//         return tcp_close_client_connection(con_state, pcb, ERR_OK);
//     }

//     return ERR_OK;
// }


// static char *strcasestr_local(const char *haystack, const char *needle) {

//     size_t needle_len = strlen(needle);

//     if (needle_len == 0) return (char *)haystack;
 
//     for (; *haystack; ++haystack) {

//         if (strncasecmp(haystack, needle, needle_len) == 0) {

//             return (char *)haystack;

//         }

//     }

//     return NULL;

// }
// static bool upload_in_progress = false;
// static int bytes = 0;
// static const char* find_eoh(const char* buf, size_t len) {
//     for (size_t i = 0; i < len - 3; i++) {
//         if (buf[i] == '\r' && buf[i+1] == '\n' && 
//             buf[i+2] == '\r' && buf[i+3] == '\n') {
//             return buf + i;
//         }
//     }
//     return NULL;
// }

// static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
//     TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T *)arg;

//     static FILE *upload_fp = NULL;
//     static size_t upload_expected = 0;
//     static size_t upload_received = 0;
//     static bool upload_in_progress = false;
//     static char header_buffer[4096];
//     static size_t header_len = 0;
//     static bool header_complete = false;

//     if (!p) {
//         DEBUG_printf("Connection closed\n");
//         if (upload_fp) {
//             fclose(upload_fp);
//             upload_fp = NULL;
//         }
//         upload_in_progress = false;
//         header_complete = false;
//         header_len = 0;
//         upload_received = 0;
//         upload_expected = 0;
//         return tcp_close_client_connection(con_state, pcb, ERR_OK);
//     }

//     tcp_recved(pcb, p->tot_len);

//     // Accumulate header until \r\n\r\n is found
//     if (!header_complete) {
//         size_t space_remaining = sizeof(header_buffer) - header_len - 1;
//         size_t to_copy = (p->tot_len < space_remaining) ? p->tot_len : space_remaining;
//         printf("p total size: %d", p->tot_len);
//         printf("Copy size for header: %d", to_copy);

//         if (to_copy > 0) {
//             pbuf_copy_partial(p, header_buffer + header_len, to_copy, 0);
//             header_len += to_copy;
//             header_buffer[header_len] = '\0';
//         }

//         const char *eoh = find_eoh(header_buffer, header_len);

//         if (eoh) {
//             header_complete = true;
//             size_t header_size = (eoh - header_buffer) + 4;

//             if (strstr(header_buffer, "POST") && strstr(header_buffer, "/upload")) {
//                 DEBUG_printf("POST /upload detected\n");

//                 char *cl = strcasestr_local(header_buffer, "Content-Length:");
//                 if (cl) {
//                     cl += 15;
//                     while (*cl == ' ') cl++;
//                     upload_expected = atoi(cl);
//                     DEBUG_printf("Content-Length: %u\n", upload_expected);
//                 }

//                 const char *filename = "/firmware.bin";
//                 upload_fp = fopen(filename, "wb");
//                 if (!upload_fp) {
//                     DEBUG_printf("FILE OPEN FAILED!\n");
//                     const char *resp = "HTTP/1.1 500 Internal Server Error\r\n\r\n";
//                     tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//                     tcp_output(pcb);
//                     pbuf_free(p);
//                     return tcp_close_client_connection(con_state, pcb, ERR_OK);
//                 }

//                 // Write body already present in header_buffer
//                 size_t body_start_offset = header_size;
//                 size_t body_len_in_header = header_len - body_start_offset;

//                 if (body_len_in_header > 0) {
//                     fwrite(header_buffer + body_start_offset, 1, body_len_in_header, upload_fp);
//                     upload_received = body_len_in_header;
//                     DEBUG_printf("Wrote %u bytes from header body\n", body_len_in_header);
//                 }

//                 upload_in_progress = true;
//             } else {
//                 DEBUG_printf("Unsupported request\n");
//                 const char *resp = "HTTP/1.1 400 Bad Request\r\n\r\n";
//                 tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//                 tcp_output(pcb);
//                 pbuf_free(p);
//                 return tcp_close_client_connection(con_state, pcb, ERR_OK);
//             }

//             pbuf_free(p);
//             return ERR_OK;
//         } else if (header_len >= sizeof(header_buffer) - 1) {
//             DEBUG_printf("Header too large!\n");
//             const char *resp = "HTTP/1.1 431 Request Header Fields Too Large\r\n\r\n";
//             tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//             tcp_output(pcb);
//             pbuf_free(p);
//             return tcp_close_client_connection(con_state, pcb, ERR_OK);
//         }

//         pbuf_free(p);
//         return ERR_OK;
//     }

//     // Write file body after header
//     if (upload_in_progress && upload_fp) {
//         struct pbuf *q = p;
//         while (q != NULL) {
//             size_t chunk_len = q->len;
//             fwrite(q->payload, 1, chunk_len, upload_fp);
//             upload_received += chunk_len;
//             q = q->next;
//         }

//         DEBUG_printf("Uploaded %u / %u bytes\n", upload_received, upload_expected);
//         pbuf_free(p);

//         if (upload_received >= upload_expected) {
//             fclose(upload_fp);
//             upload_fp = NULL;
//             upload_in_progress = false;
//             header_complete = false;
//             header_len = 0;
//             upload_expected = 0;
//             upload_received = 0;

//             const char *resp = "HTTP/1.1 200 OK\r\n\r\n";
//             tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//             tcp_output(pcb);
//             DEBUG_printf("Upload complete\n");
//         }

//         return ERR_OK;
//     }

//     pbuf_free(p);
//     return ERR_OK;
// }

//completely died code, cannot send any of them, nor header nor data completelty

// static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
//     TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
 
//     static FILE *upload_fp = NULL;
//     static int upload_expected = 0;
//     static int upload_received = 0;
 
//     if (!p) {
//         DEBUG_printf("connection closed\n");
//         if (upload_fp) {
//             fclose(upload_fp);
//             upload_fp = NULL;
//         }
//         return tcp_close_client_connection(con_state, pcb, ERR_OK);
//     }

//     printf("Recieved data length: %d", p->tot_len);
 
//     assert(con_state && con_state->pcb == pcb);
 
//     // If we're in the middle of receiving file body
//     if (upload_fp && upload_received < upload_expected) {
//         fwrite(p->payload, 1, p->tot_len, upload_fp);
//         upload_received += p->tot_len;
 
//         tcp_recved(pcb, p->tot_len);
//         pbuf_free(p);
 
//         if (upload_received >= upload_expected) {
//             fclose(upload_fp);
//             upload_fp = NULL;
//             upload_in_progress = false;
 
//             const char *resp = "HTTP/1.1 200 OK\r\nContent-Length:0\r\n\r\n";
//             tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//             tcp_output(pcb);
//             return tcp_close_client_connection(con_state, pcb, ERR_OK);
//         }
//         return ERR_OK;
//     }
 
//     if (p->tot_len > 0) {
//         DEBUG_printf("tcp_server_recv %d err %d\n", p->tot_len, err);
 
//         if(!upload_in_progress){
//             pbuf_copy_partial(p, con_state->headers,
//                           p->tot_len > sizeof(con_state->headers) - 1 ?
//                           sizeof(con_state->headers) - 1 : p->tot_len, 0);
//         }
//         if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
//             char *request = con_state->headers + sizeof(HTTP_GET);
//             char *params = strchr(request, '?');
 
//             if (params) {
//                 if (*params) {
//                     char *space = strchr(request, ' ');
//                     *params++ = 0;
//                     if (space) *space = 0;
//                 } else {
//                     params = NULL;
//                 }
//             }
 
//             con_state->result_len = test_server_content(request, params, con_state->result, sizeof(con_state->result));
//             DEBUG_printf("Request: %s\n", request);
//             DEBUG_printf("Result Length: %d\n", con_state->result_len);
 
//             if (con_state->result_len > sizeof(con_state->result) - 1) {
//                 DEBUG_printf("Too much result data %d\n", con_state->result_len);
//                 return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
//             }
 
//             con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers),
//                                              HTTP_RESPONSE_HEADERS, 200, con_state->result_len);
    
//             if (con_state->header_len > sizeof(con_state->headers) - 1) {
//                 DEBUG_printf("Too much header data %d\n", con_state->header_len);
//                 return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
//             }
 
//             char *full_response = malloc(con_state->header_len + con_state->result_len);
//             if (!full_response) {
//                 DEBUG_printf("Failed to allocate full response buffer\n");
//                 return tcp_close_client_connection(con_state, pcb, ERR_MEM);
//             }
 
//             memcpy(full_response, con_state->headers, con_state->header_len);
//             memcpy(full_response + con_state->header_len, con_state->result, con_state->result_len);
 
//             con_state->sent_len = 0;
//             err_t write_err = tcp_write(pcb, full_response, con_state->header_len + con_state->result_len, 0);
//             tcp_output(pcb);
 
//             free(full_response);
 
//             if (write_err != ERR_OK) {
//                 DEBUG_printf("failed to write full response %d\n", write_err);
//                 return tcp_close_client_connection(con_state, pcb, write_err);
//             }
//         } else if (strncmp(con_state->headers, "POST", 4) == 0 && strstr(con_state->headers, "/upload") != NULL) {
//             DEBUG_printf("Received a POST /upload request\n");
 
//             char *cl_start = strcasestr_local(con_state->headers, "Content-Length:");
//             upload_expected = 0;
//             if (cl_start) {
//                 cl_start += strlen("Content-Length:");
//                 while (*cl_start == ' ') cl_start++;
//                 upload_expected = atoi(cl_start);
//             }
 
//             char *body = strstr(con_state->headers, "\r\n\r\n");
//             if (body) {
//                 body += 4; // skip \r\n\r\n
//                 upload_received = p->tot_len - (body - con_state->headers);
//                 upload_fp = fopen("firmware.bin", "wb");
//                 if (upload_fp && upload_received > 0) {
//                     fwrite(body, 1, upload_received, upload_fp);
//                     DEBUG_printf("Wrote initial %d bytes\n", upload_received);
//                 }
//                 upload_in_progress = true;
//                 if (upload_received >= upload_expected) {
//                     fclose(upload_fp);
//                     upload_fp = NULL;
//                     upload_in_progress = false;
//                     const char *response = "HTTP/1.1 200 OK\r\nContent-Length:0\r\n\r\n";
//                     tcp_write(pcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
//                     tcp_output(pcb);
//                     return tcp_close_client_connection(con_state, pcb, ERR_OK);
//                 }
//             }
//         }
 
//         tcp_recved(pcb, p->tot_len);
//     }
 
//     pbuf_free(p);
//     return ERR_OK;
// }

//hopeless 1: Header isnt coming but complete data is coming
static int total = 0;
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb,
                             struct pbuf *p, err_t err)
{
    TCP_CONNECT_STATE_T *st = (TCP_CONNECT_STATE_T *)arg;
 
    // --- Static upload-session state ---
    static bool header_done = false;       // true once we saw the blank line
    static int bytes_in_body = 0;          // counts file bytes
    static int expected_content_length = 0; // from Content-Length header
    static bool response_sent = false;      // track if we've sent response
    static char boundary[128] = {0};        // store boundary string
    static bool boundary_found = false;
    // -----------------------------------
 
    if (!p) {
        DEBUG_printf("\n[+] Client closed connection.\n");
        DEBUG_printf("[+] Total received body bytes: %d\n", bytes_in_body);
        DEBUG_printf("[+] Expected content length: %d\n", expected_content_length);
 
        // Reset session state
        header_done = false;
        bytes_in_body = 0;
        expected_content_length = 0;
        response_sent = false;
        boundary_found = false;
        memset(boundary, 0, sizeof(boundary));
 
        return tcp_close_client_connection(st, pcb, ERR_OK);
    }
 
    assert(st && st->pcb == pcb);
    tcp_recved(pcb, p->tot_len);  // tell lwIP we've consumed this many bytes
 
    DEBUG_printf("\n[>] Received pbuf with total length: %d bytes\n", p->tot_len);
 
    struct pbuf *q = p;
    while (q) {
        const char *buf = (const char *)q->payload;
        int len = q->len;
 
        DEBUG_printf("[>] Segment length: %d bytes\n", len);
 
        if (!header_done) {
            DEBUG_printf("[~] Scanning for end of HTTP headers...\n");
            
            // Look for Content-Length in headers
            if (!expected_content_length) {
                char *content_len_pos = strstr(buf, "Content-Length: ");
                if (content_len_pos) {
                    expected_content_length = atoi(content_len_pos + 16);
                    DEBUG_printf("[!] Found Content-Length: %d\n", expected_content_length);
                }
            }
            
            // Look for boundary in headers
            if (!boundary_found) {
                char *boundary_pos = strstr(buf, "boundary=");
                if (boundary_pos) {
                    char *boundary_start = boundary_pos + 9;
                    char *boundary_end = strstr(boundary_start, "\r\n");
                    if (boundary_end) {
                        int boundary_len = boundary_end - boundary_start;
                        if (boundary_len < sizeof(boundary) - 1) {
                            memcpy(boundary, boundary_start, boundary_len);
                            boundary[boundary_len] = '\0';
                            boundary_found = true;
                            DEBUG_printf("[!] Found boundary: %s\n", boundary);
                        }
                    }
                }
            }
 
            for (int i = 0; i < len; i++) {
                bool crlf = (buf[i]=='\r' && i+3 < len &&
                             buf[i+1]=='\n' && buf[i+2]=='\r' && buf[i+3]=='\n');
                bool lf = (buf[i]=='\n' && i+1 < len &&
                           buf[i+1]=='\n');
 
                if (crlf || lf) {
                    int hdr_end = i + (crlf ? 4 : 2);
                    header_done = true;
 
                    DEBUG_printf("[✓] HTTP header end detected at byte offset %d\n", hdr_end);
                    DEBUG_printf("[#] ---- HTTP HEADER START ----\n");
                    fwrite(buf, 1, hdr_end, stdout);
                    DEBUG_printf("[#] ---- HTTP HEADER END ----\n");
 
                    // Handle remaining body bytes in same segment
                    if (hdr_end < len) {
                        int body_here = len - hdr_end;
                        DEBUG_printf("[↓] Initial body data (after header): %d bytes\n", body_here);
                        fwrite(buf + hdr_end, 1, body_here, stdout);
                        bytes_in_body += body_here;
                    }
 
                    goto next_segment;
                }
            }
 
            DEBUG_printf("[!] End of header not found yet. Printing entire segment:\n");
            fwrite(buf, 1, len, stdout);
        }
        else {
            DEBUG_printf("[↓] Receiving file body: %d bytes (total so far: %d/%d)\n", 
                        len, bytes_in_body + len, expected_content_length);
            fwrite(buf, 1, len, stdout);
            bytes_in_body += len;
            
            // Check if we've received all expected data
            if (bytes_in_body >= expected_content_length) {
                DEBUG_printf("[✓] All data received! Sending response...\n");
                
                if (!response_sent) {
                    const char *resp = "HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n";
                    tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
                    tcp_output(pcb);
                    response_sent = true;
                    DEBUG_printf("[→] Sent HTTP 200 OK response to client\n");
                }
                
                // Close connection after sending response
                pbuf_free(p);
                return tcp_close_client_connection(st, pcb, ERR_OK);
            }
        }
 
    next_segment:
        q = q->next;
    }
 
    DEBUG_printf("Total length is: %d\n", bytes_in_body);
    pbuf_free(p);
    return ERR_OK;  // keep connection open until all data received
}
//hopeless code 2: Header is getting, but complete data isnt coming

// static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb,
//                              struct pbuf *p, err_t err)
// {
//     TCP_CONNECT_STATE_T *st = (TCP_CONNECT_STATE_T *)arg;
 
//     /* -------- persistent, per‑connection state -------- */
//     static bool  header_done   = false;
//     static int   bytes_in_body = 0;
//     static int   content_len   = -1;          /* ‑1 = unknown               */
//     static char  hdr_buf[2048];                /* accumulates split headers  */
//     static int   hdr_len = 0;
//     /* --------------------------------------------------- */
 
//     /* client closed socket */
//     if (!p) {
//         DEBUG_printf("\n[+] Client closed. body=%d bytes\n", bytes_in_body);
//         header_done   = false;
//         bytes_in_body = 0;
//         content_len   = -1;
//         hdr_len       = 0;
//         return tcp_close_client_connection(st, pcb, ERR_OK);
//     }
 
//     assert(st && st->pcb == pcb);
//     tcp_recved(pcb, p->tot_len);
 
//     struct pbuf *seg = p;
//     while (seg) {
//         const char *buf = (const char *)seg->payload;
//         int len = seg->len;
//         total+=len;
 
//         DEBUG_printf("[>] seg %d bytes  (header_done=%d)\n", len, header_done);
 
//         /* ---------- HEADER PHASE ---------- */
//         if (!header_done) {
//             /* 1. copy into hdr_buf (but don’t overflow) */
//             int copy = MIN(len, (int)sizeof(hdr_buf) - hdr_len - 1);
//             memcpy(hdr_buf + hdr_len, buf, copy);
//             hdr_len += copy;
//             hdr_buf[hdr_len] = '\0';
 
//             /* 2. look for delimiter in combined buffer */
//             char *delim = strstr(hdr_buf, "\r\n\r\n");
//             if (!delim) delim = strstr(hdr_buf, "\n\n");
 
//             if (!delim) {
//                 /* header still incomplete */
//                 DEBUG_printf("[~] header not finished yet (%d bytes)\n", hdr_len);
//                 seg = seg->next;
//                 continue;
//             }
 
//             /* 3. header now complete */
//             int header_bytes = (delim - hdr_buf) + ((*delim == '\r') ? 4 : 2);
//             header_done = true;
//             DEBUG_printf("[✓] header complete (%d bytes)\n", header_bytes);
//             fwrite(hdr_buf, 1, header_bytes, stdout);
//             DEBUG_printf("[#] ---- HEADER END ----\n");
 
//             /* parse Content‑Length (optional) */
//             char *cl = strcasestr_local(hdr_buf, "Content-Length:");
//             if (cl) {
//                 cl += 15; while (*cl == ' ') cl++;
//                 content_len = atoi(cl);
//                 DEBUG_printf("[i] Content-Length: %d\n", content_len);
//             }
 
//             /* send HTTP 200 OK immediately */
//             const char *resp = "HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n";
//             tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//             tcp_output(pcb);
 
//             /* 4. anything after header in hdr_buf is body */
//             int start_body = header_bytes;
//             int remain_hdr_buf = hdr_len - start_body;
//             if (remain_hdr_buf > 0) {
//                 fwrite(hdr_buf + start_body, 1, remain_hdr_buf, stdout);
//                 bytes_in_body += remain_hdr_buf;
//             }
 
//             /* 5. anything after COPY in current pbuf is body too */
//             int remain_seg = len - copy;
//             if (remain_seg > 0) {
//                 fwrite(buf + copy, 1, remain_seg, stdout);
//                 bytes_in_body += remain_seg;
//             }
 
//             hdr_len = 0;           /* ready for next connection */
//             seg = seg->next;
//             continue;
//         }
 
//         /* ---------- BODY PHASE ---------- */
//         fwrite(buf, 1, len, stdout);
//         bytes_in_body += len;
 
//         /* if we know Content-Length, close when done */
//         if (content_len > 0 && bytes_in_body >= content_len) {
//             DEBUG_printf("[+] full body received (%d bytes)\n", bytes_in_body);
//             tcp_close_client_connection(st, pcb, ERR_OK);
//             header_done   = false;
//             bytes_in_body = 0;
//             content_len   = -1;
//             hdr_len       = 0;
//             pbuf_free(p);
//             return ERR_OK;
//         }
 
//         seg = seg->next;
//     }
//     printf("Total length is: %d", total);
//     pbuf_free(p);
//     return ERR_OK;  /* keep connection until FIN or content_len satisfied */
// }

// static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
// {
//     TCP_CONNECT_STATE_T *st = (TCP_CONNECT_STATE_T *)arg;
 
//     // --- Static upload-session state ---
//     static bool header_done = false;
//     static int bytes_in_body = 0;
//     static int expected_content_length = 0;
//     static char header_buffer[2048];
//     static int header_len = 0;
//     // -----------------------------------
 
//     if (!p) {
//         DEBUG_printf("\n[+] Client closed connection.\n");
//         DEBUG_printf("[+] Total body bytes received: %d / %d\n", bytes_in_body, expected_content_length);
 
//         // Reset state
//         header_done = false;
//         bytes_in_body = 0;
//         expected_content_length = 0;
//         header_len = 0;
 
//         return tcp_close_client_connection(st, pcb, ERR_OK);
//     }
 
//     assert(st && st->pcb == pcb);
//     tcp_recved(pcb, p->tot_len);
 
//     struct pbuf *q = p;
//     while (q) {
//         const char *buf = (const char *)q->payload;
//         int len = q->len;
 
//         DEBUG_printf("\n[>] seg %d bytes  (header_done=%d)\n", len, header_done);
 
//         if (!header_done) {
//             // Check if we have enough space in header buffer
//             int copy = MIN(len, (int)sizeof(header_buffer) - header_len - 1);
//             memcpy(header_buffer + header_len, buf, copy);
//             header_len += copy;
//             header_buffer[header_len] = '\0';
 
//             // Search for end of header
//             char *body_start = NULL;
//             if ((body_start = strstr(header_buffer, "\r\n\r\n")) || (body_start = strstr(header_buffer, "\n\n"))) {
//                 body_start += (body_start[1] == '\n') ? 2 : 4; // skip \r\n\r\n or \n\n
//                 int hdr_size = body_start - header_buffer;
//                 int body_in_header = header_len - hdr_size;
 
//                 DEBUG_printf("[ ] header complete (%d bytes)\n", hdr_size);
//                 fwrite(header_buffer, 1, hdr_size, stdout);
 
//                 // Parse Content-Length
//                 const char *cl = strstr(header_buffer, "Content-Length:");
//                 if (cl) {
//                     cl += strlen("Content-Length:");
//                     while (*cl == ' ') cl++;
//                     expected_content_length = atoi(cl);
//                     DEBUG_printf("[i] Content-Length: %d\n", expected_content_length);
//                 }
 
//                 DEBUG_printf("[#] ---- HEADER END ----\n");
 
//                 header_done = true;
 
//                 // Send 200 OK response
//                 const char *resp = "HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n";
//                 tcp_write(pcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
//                 tcp_output(pcb);
//                 DEBUG_printf("[→] Sent HTTP 200 OK response\n");
 
//                 // Print body already in this pbuf
//                 if (body_in_header > 0) {
//                     fwrite(body_start, 1, body_in_header, stdout);
//                     bytes_in_body += body_in_header;
//                 }
 
//             } else {
//                 DEBUG_printf("[~] header not finished yet (%d bytes)\n", header_len);
//             }
 
//         } else {
//             // Already in body phase
//             DEBUG_printf("[↓] Receiving file body: %d bytes\n", len);
//             fwrite(buf, 1, len, stdout);
//             bytes_in_body += len;
//         }
 
//         // Check if all data is received
//         if (header_done && expected_content_length > 0 && bytes_in_body >= expected_content_length) {
//             DEBUG_printf("[✓] Upload complete! Total bytes received = %d\n", bytes_in_body);
 
//             // Reset state
//             header_done = false;
//             bytes_in_body = 0;
//             expected_content_length = 0;
//             header_len = 0;
 
//             pbuf_free(p);
//             return tcp_close_client_connection(st, pcb, ERR_OK);
//         }
 
//         q = q->next;
//     }
 
//     DEBUG_printf("[i] Total so far: %d / %d\n", bytes_in_body, expected_content_length);
//     pbuf_free(p);
//     return ERR_OK;
// }

 /**
  * @brief TCP poll callback function
  * 
  * Called periodically to check connection status.
  * 
  * @param arg Connection state structure
  * @param pcb TCP protocol control block
  * @return Error code
  */
 static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) {
     TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    //  DEBUG_printf("tcp_server_poll_fn\n");
     return tcp_close_client_connection(con_state, pcb, ERR_OK);
 }
 
 /**
  * @brief TCP error callback function
  * 
  * Called when an error occurs on a connection.
  * 
  * @param arg Connection state structure
  * @param err Error code
  */
 static void tcp_server_err(void *arg, err_t err) {
     TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
     if (err != ERR_ABRT) {
         DEBUG_printf("tcp_client_err_fn %d\n", err);
         tcp_close_client_connection(con_state, con_state->pcb, err);
     }
 }
 
 /**
  * @brief TCP accept callback function
  * 
  * Called when a new client connection is accepted.
  * 
  * @param arg Server state structure
  * @param client_pcb Client TCP protocol control block
  * @param err Error code
  * @return Error code
  */
 static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
     TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
     
     // Check for errors
     if (err != ERR_OK || client_pcb == NULL) {
         DEBUG_printf("failure in accept\n");
         return ERR_VAL;
     }
     client_connected = true;
     DEBUG_printf("client connected\n");
     
     // Update connection state - client connected
     update_connection_state(true);
 
     // Allocate connection state
     TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
     if (!con_state) {
         DEBUG_printf("failed to allocate connect state\n");
         return ERR_MEM;
     }

     con_state->pcb = client_pcb;
     connected_client_pcb = client_pcb;
     con_state->gw = &state->gw;
 
     // Set callbacks for the connection
     tcp_arg(client_pcb, con_state);
     tcp_sent(client_pcb, tcp_server_sent);
     tcp_recv(client_pcb, tcp_server_recv);
     tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
     tcp_err(client_pcb, tcp_server_err);
 
     return ERR_OK;
 }
 
 /**
  * @brief Opens the TCP server
  * 
  * @param arg Server state structure
  * @param ap_name Access point name
  * @return true if successful, false otherwise
  */
 static bool tcp_server_open(void *arg, const char *ap_name) {
     TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
     DEBUG_printf("starting server on port %d\n", TCP_PORT);
  
     // Create a new TCP PCB
     struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
     if (!pcb) {
         DEBUG_printf("failed to create pcb\n");
         return false;
     }
  
     // Bind to the specified port
     err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
     if (err) {
         DEBUG_printf("failed to bind to port %d\n", TCP_PORT);
         return false;
     }
  
     // Start listening for connections
     state->server_pcb = tcp_listen_with_backlog(pcb, 1);
     if (!state->server_pcb) {
         DEBUG_printf("failed to listen\n");
         if (pcb) {
             tcp_close(pcb);
         }
         return false;
     }
  
     // Set callback for accepting new connections
     tcp_arg(state->server_pcb, state);
     tcp_accept(state->server_pcb, tcp_server_accept);
  
     printf("Try connecting to '%s' (press 'd' to disable access point)\n", ap_name);
     return true;
 }

void ble_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    uint8_t event_type = hci_event_packet_get_type(packet);
    uint16_t mtu = att_event_mtu_exchange_complete_get_MTU(packet);
    // printf("BLE Event received: 0x%02X\n", event_type);

    switch (event_type) {
        case HCI_EVENT_LE_META:
            if (packet[2] == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                con_handle = little_endian_read_16(packet, 4);
                if(con_handle != HCI_CON_HANDLE_INVALID) {
                    ble_connected = true;
                    power_state.last_activity = get_absolute_time(); // Update activity
                    uart_puts(UART_ID, "BLE connected\n");
                    printf("Connected with handle: 0x%04X\n", con_handle);
                } else {
                    printf("Connection failed!\n");
                }
                break;
            }
            break;

            case BTSTACK_EVENT_STATE:
            if(btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                printf("HCI State: Working - Starting advertisements\n");
                uint8_t name_len = strlen(DEVICE_NAME);
                data[0] = name_len+1;
                data[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
                memcpy(&data[2], DEVICE_NAME, name_len);

                gap_advertisements_set_params(0x0020, 0x0020, 0, 0, NULL, 0x07, 0x00);
                gap_advertisements_set_data(sizeof(adv_data),(uint8_t *) adv_data);
                gap_scan_response_set_data(name_len+2, data);
                gap_advertisements_enable(1);
            }
            break;

        case BTSTACK_EVENT_NR_CONNECTIONS_CHANGED:
            printf("Connection count changed to: %d\n", packet[2]);
            if(packet[2] == 1){
                ble_connected = true;
                power_state.last_activity = get_absolute_time();
                printf("Connection established - starting key validation\n");
                validate_key.process = &key_handler;
                btstack_run_loop_set_timer(&validate_key, KEY_INTERVAL);
                btstack_run_loop_add_timer(&validate_key);
            }
            else if (packet[2] == 0){
                uart_puts(UART_ID, "Disconnected\n");
                printf("Disconnected\n");
                handle_disconnection();
            }
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            handle_att_can_send_now_event();
            break;

        case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
            
            printf("Current MTU: %d bytes\n", mtu);
            printf("Max notification size: %d bytes\n", mtu - 3);
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("HCI Disconnection complete\n");
            ble_done = true;
            // handle_disconnection();
            // cyw43_arch_deinit();
            
            
            break;
    }
}

void ble_mode(){
    if(cyw43_arch_init()){
        printf("BLE init failed\n");
        ble_done = true;
        return ;
    }

    led_state = false;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

    l2cap_init();
    sm_init();
    att_server_init(profile_data, NULL, att_write_callback);
    att_server_register_packet_handler(ble_event_handler);

    hci_event_callback_registration.callback = &ble_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    power_state.last_activity = get_absolute_time();
    power_state.client_connected = false;
    power_state.sleep_mode_active = false;

    memset(can_batch_response, 0, sizeof(can_batch_response));
    last_clear_time = get_absolute_time();
    buffer_full_reported = false;

    add_repeating_timer_ms(1000, power_management_callback, NULL, &power_management_timer); 
    add_repeating_timer_ms(1, can_check_callback, NULL, &can_check_timer);
    
    printf("CAN timer started...\n");

    hci_power_control(HCI_POWER_ON);
    printf("BLE is active and advertising!\n");
    ble_done = false;

    btstack_run_loop_execute();

    printf("BLE session ended.\n");

    cancel_repeating_timer(&power_management_timer);
    cancel_repeating_timer(&can_check_timer);
    hci_power_control(HCI_POWER_OFF);
    cyw43_arch_deinit();

    if(current_mode == MODE_WIFI_AP) wifi_ap_mode();
}

void wifi_ap_mode(){
    printf("Entered in wifi mode NOICE :)");
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        printf("failed to allocate state\n");
        return;
    }

    // Initialize wireless chip
    if (cyw43_arch_init()) {
        printf("Wifi init failed\n");
        return;
    }

    // Enable power saving
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);

    // Initialize CAN controller
    mcp2515_init();

    // Configure access point
    const char *ap_name = "KION_Truck1";
    const char *password = "Kion@123";

    // Enable access point mode with WPA2 security
    cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

    // Configure network addresses
    ip4_addr_t mask;
    IP4_ADDR(ip_2_ip4(&state->gw), 192, 168, 4, 1);
    IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

    // Initialize DHCP server
    dhcp_server_t dhcp_server;
    dhcp_server_init(&dhcp_server, &state->gw, &mask);

    // Initialize DNS server
    dns_server_t dns_server;
    dns_server_init(&dns_server, &state->gw);

    // Open TCP server
    if (!tcp_server_open(state, ap_name)) {
        printf("failed to open server\n");
        return;
    }

    isSwitched = true;

    const char *ack = "BLE switched to Wi-Fi successfully.";
    if (connected_client_pcb && client_connected) { 
    err_t err = tcp_write(connected_client_pcb, ack, strlen(ack), TCP_WRITE_FLAG_COPY);
    tcp_output(connected_client_pcb);
    if (err != ERR_OK) {
        printf("Failed to send switch ACK: %d\n", err);
    } else {
        printf("Sent switch acknowledgment to client.\n");
        }
    }

    // Track timing
    last_clear_time = get_absolute_time();
    power_state.last_activity = get_absolute_time();
    enter_sleep_mode();

    // Set up timers to avoid polling
    add_repeating_timer_ms(2000, power_management_callback, NULL, &power_management_timer);
    add_repeating_timer_ms(200, can_check_callback, NULL, &can_check_timer);

    // Optional: GPIO interrupt for AP control
    // gpio_set_irq_enabled_with_callback(PICO_DEFAULT_LED_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Main loop — no polling or sleep
    while (!state->complete) {
       // Wait for events efficiently without polling
       __wfi();  // Wait For Interrupt - puts CPU to sleep until an interrupt occurs
        
       // Check for key presses - only when event occurs
       int key = getchar_timeout_us(0);
       if (key == 'd' || key == 'D') {
           printf("Disabling Access Point mode\n");
           cyw43_arch_lwip_begin();
           cyw43_arch_disable_ap_mode();
           cyw43_arch_lwip_end();
           state->complete = true;
       }
    }

    cancel_repeating_timer(&power_management_timer);
    cancel_repeating_timer(&can_check_timer);
    tcp_server_close(state);
    dns_server_deinit(&dns_server);
    dhcp_server_deinit(&dhcp_server);
    cyw43_arch_deinit();
    free(state);
}

int main() {
    stdio_init_all();

    printf("Starting the application...\n");

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false);

    printf("Initializing CAN...\n");
    if(!init_can_system()){
        printf("CAN initialization failed!\n");
    }else{
        printf("CAN initializated successfully\n");
    }

    ble_mode();

    return 0;
}
