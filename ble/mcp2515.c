// mcp2515.c - Driver for MCP2515 CAN Bus Controller
// Provides interface functions for initializing and communicating with MCP2515 via SPI
// Optimized for J1939 protocol (250kbps with 29-bit extended IDs)

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "mcp2515.h"
#include "pico/time.h"

/**
 * Activates the chip select line to begin SPI communication
 */
void cs_select() {
    gpio_put(PIN_CS, 0);  // Active low
}

/**
 * Deactivates the chip select line to end SPI communication
 */
void cs_deselect() {
    gpio_put(PIN_CS, 1);  // Inactive high
}

/**
 * Writes a value to a specific register in the MCP2515
 * 
 * @param address Register address to write to
 * @param value Value to write to the register
 */
void mcp2515_write_register(uint8_t address, uint8_t value) {
    cs_select();
    uint8_t buf[] = {MCP2515_WRITE, address, value};  // Command, address, data
    spi_write_blocking(MCP2515_SPI_PORT, buf, 3);
    cs_deselect();
}

/**
 * Reads a value from a specific register in the MCP2515
 * 
 * @param address Register address to read from
 * @return The value read from the register
 */
uint8_t mcp2515_read_register(uint8_t address) {
    cs_select();
    uint8_t buf[] = {MCP2515_READ, address, 0x00};  // Command, address, dummy byte
    uint8_t rx_buf[3];
    spi_write_read_blocking(MCP2515_SPI_PORT, buf, rx_buf, 3);
    cs_deselect();
    return rx_buf[2];  // Data is in the third byte
}

/**
 * Reads multiple consecutive registers from the MCP2515
 * 
 * @param address Starting register address
 * @param values Buffer to store read values
 * @param count Number of registers to read
 */
void mcp2515_read_registers(uint8_t address, uint8_t *values, uint8_t count) {
    cs_select();
    uint8_t buf[2] = {MCP2515_READ, address};  // Command, starting address
    spi_write_blocking(MCP2515_SPI_PORT, buf, 2);
    spi_read_blocking(MCP2515_SPI_PORT, 0, values, count);  // Read consecutive registers
    cs_deselect();
}

/**
 * Modifies specific bits in a register without affecting other bits
 * 
 * @param address Register address to modify
 * @param mask Bit mask - only bits set to 1 will be affected
 * @param data New values for the bits (applied where mask is 1)
 */
void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data) {
    cs_select();
    uint8_t buf[] = {MCP2515_BIT_MODIFY, address, mask, data};
    spi_write_blocking(MCP2515_SPI_PORT, buf, 4);
    cs_deselect();
}

/**
 * Gets the current status of the MCP2515
 * 
 * @return Status byte with interrupt and buffer state information
 */
uint8_t mcp2515_get_status() {
    cs_select();
    uint8_t buf[] = {MCP2515_STATUS, 0x00};  // Status command + dummy byte
    uint8_t status[2];
    spi_write_read_blocking(MCP2515_SPI_PORT, buf, status, 2);
    cs_deselect();
    return status[1];  // Status is in the second byte
}

/**
 * Sets the operating mode of the MCP2515
 * 
 * @param mode The mode to set (MODE_NORMAL, MODE_CONFIG, etc.)
 * @return true if mode was set successfully, false if timed out
 */
bool mcp2515_set_mode(uint8_t mode) {
    // Modify only the mode bits in CANCTRL
    mcp2515_bit_modify(CANCTRL, MODE_MASK, mode);
    
    // Wait for mode change to complete (check CANSTAT)
    uint8_t timeout = 50; // ~50ms timeout
    while (timeout--) {
        if ((mcp2515_read_register(CANSTAT) & MODE_MASK) == mode) {
            return true;  // Mode change successful
        }
        sleep_ms(1);
    }
    return false;  // Mode change timed out
}

/**
 * Initializes the MCP2515 CAN controller for J1939 communication
 * 
 * @return true if initialization was successful, false otherwise
 */
bool mcp2515_init() {
    // Initialize SPI with appropriate settings - lower speed for reliability
    spi_init(MCP2515_SPI_PORT, 1000000); // 1MHz SPI clock
    
    // Configure SPI GPIO pins
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Configure chip select pin
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);  // Deselect initially (high)
    
    // Configure interrupt pin with pull-up
    gpio_init(PIN_INT);
    gpio_set_dir(PIN_INT, GPIO_IN);
    gpio_pull_up(PIN_INT);  // INT is active low
    
    // Perform hardware reset of MCP2515
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &(uint8_t){MCP2515_RESET}, 1);
    cs_deselect();
    sleep_ms(10);  // Wait for reset to complete
    
    // Enter config mode - required for changing settings
    if (!mcp2515_set_mode(MODE_CONFIG)) {
        printf("Failed to enter config mode\n");
        return false;
    }

    // Configure bit timing for 250Kbps with 8MHz oscillator (J1939 standard)
    // These values must be calculated based on oscillator frequency and desired bit rate
    mcp2515_write_register(CNF1, 0x01); // BRP=1, TQ=2
    mcp2515_write_register(CNF2, 0xB8); // PS1=7, PRSEG=1, SJW=3
    mcp2515_write_register(CNF3, 0x05); // PS2=6
    
    // Configure reception - accept all messages
    mcp2515_write_register(RXB0CTRL, 0x60); // Receive any message, disable filters
    mcp2515_write_register(RXB1CTRL, 0x60); // Receive any message, disable filters
    
    // Enable RX interrupts for both buffers
    mcp2515_write_register(CANINTE, 0x03); // Enable RXB0 and RXB1 interrupts
    
    // Enter normal operation mode
    if (!mcp2515_set_mode(MODE_NORMAL)) {
        printf("Failed to enter normal mode\n");
        return false;
    }
    
    printf("MCP2515 initialized successfully for J1939 protocol\n");
    return true;
}

/**
 * Checks if any messages are available to be received
 * 
 * @return true if a message is available, false otherwise
 */
bool mcp2515_check_receive() {
    // Check status register for RX flags
    uint8_t status = mcp2515_get_status();
    bool has_message = ((status & (STAT_RX0IF | STAT_RX1IF)) != 0);

    // Also check the interrupt pin if available
    // (LOW indicates message available since we have a pull-up)
    if (!has_message && PIN_INT != -1) {
        has_message = !gpio_get(PIN_INT);
    }
    
    return has_message;
}

/**
 * Receives a CAN message if available
 * 
 * @param msg Pointer to CANMessage structure to fill with received data
 * @return true if a message was received, false otherwise
 */
// Increase SPI speed to 10MHz in init function
// bool mcp2515_init() {
//     spi_init(MCP2515_SPI_PORT, 10 * 1000 * 1000); // 10MHz SPI clock
    
//     gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
//     gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
//     gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
//     // Configure chip select pin
//     gpio_init(PIN_CS);
//     gpio_set_dir(PIN_CS, GPIO_OUT);
//     gpio_put(PIN_CS, 1);  // Deselect initially (high)
    
//     // Configure interrupt pin with pull-up
//     gpio_init(PIN_INT);
//     gpio_set_dir(PIN_INT, GPIO_IN);
//     gpio_pull_up(PIN_INT);  // INT is active low
    
//     // Perform hardware reset of MCP2515
//     cs_select();
//     spi_write_blocking(MCP2515_SPI_PORT, &(uint8_t){MCP2515_RESET}, 1);
//     cs_deselect();
//     sleep_ms(10);  // Wait for reset to complete
    
//     // Enter config mode - required for changing settings
//     if (!mcp2515_set_mode(MODE_CONFIG)) {
//         printf("Failed to enter config mode\n");
//         return false;
//     }

//     // Configure bit timing for 250Kbps with 8MHz oscillator (J1939 standard)
//     // These values must be calculated based on oscillator frequency and desired bit rate
//     mcp2515_write_register(CNF1, 0x01); // BRP=1, TQ=2
//     mcp2515_write_register(CNF2, 0xB8); // PS1=7, PRSEG=1, SJW=3
//     mcp2515_write_register(CNF3, 0x05); // PS2=6
    
//     // Configure reception - accept all messages
//     mcp2515_write_register(RXB0CTRL, 0x60); // Receive any message, disable filters
//     mcp2515_write_register(RXB1CTRL, 0x60); // Receive any message, disable filters
    
//     // Enable RX interrupts for both buffers
//     mcp2515_write_register(CANINTE, 0x03); // Enable RXB0 and RXB1 interrupts
    
//     // Enter normal operation mode
//     if (!mcp2515_set_mode(MODE_NORMAL)) {
//         printf("Failed to enter normal mode\n");
//         return false;
//     }
    
//     printf("MCP2515 initialized successfully for J1939 protocol\n");
//     return true;
// }

// Optimized receive function
bool mcp2515_receive_message(CANMessage *msg) {
    uint8_t status = mcp2515_get_status();
    uint8_t buffer_index = 0;
    
    // Combined buffer processing
    if (status & (STAT_RX0IF | STAT_RX1IF)) {
        uint8_t reg_base = (status & STAT_RX0IF) ? RXB0SIDH : RXB1SIDH;
        uint8_t buffer[14];
        mcp2515_read_registers(reg_base, buffer, 14);

        // Message extraction
        bool ext_id = (buffer[1] & 0x08) != 0;
        if (ext_id) {
            msg->id = ((uint32_t)buffer[0] << 21) |
                     ((uint32_t)(buffer[1] & 0xE0) << 13) |
                     ((uint32_t)(buffer[1] & 0x03) << 16) |
                     ((uint32_t)buffer[2] << 8) |
                     ((uint32_t)buffer[3]);
        } else {
            msg->id = ((uint32_t)buffer[0] << 3) | (buffer[1] >> 5);
        }
        
        msg->dlc = buffer[4] & 0x0F;
        if (msg->dlc > 8) msg->dlc = 8;
        
        for (int i = 0; i < msg->dlc; i++) {
            msg->data[i] = buffer[5 + i];
        }
        
        // Clear interrupt flag
        mcp2515_bit_modify(CANINTF, (status & STAT_RX0IF) ? STAT_RX0IF : STAT_RX1IF, 0);
        return true;
    }
    return false;
}

void mcp2515_check_and_clear_errors() {
    uint8_t eflg = mcp2515_read_register(EFLG);
    
    // Check for RX buffer overflow
    if (eflg & (0x40 | 0x80)) { // RX0OVR or RX1OVR
        printf("CAN RX Buffer overflow detected! EFLG=0x%02X\n", eflg);
        
        // Clear overflow flags
        mcp2515_bit_modify(EFLG, 0xC0, 0); // Clear RX0OVR and RX1OVR
        
        // Clear any pending interrupt flags
        mcp2515_bit_modify(CANINTF, 0xFF, 0);
    }
    
    // Check for other error conditions
    if (eflg & 0x3F) { // Any other error flags
        printf("CAN Error flags: 0x%02X\n", eflg);
    }
}

/**
 * Enhanced receive function that handles both buffers more efficiently
 */
bool mcp2515_receive_all_pending(CANMessage *messages, uint8_t max_messages, uint8_t *received_count) {
    *received_count = 0;
    uint8_t status = mcp2515_get_status();
    
    // Combined buffer processing
    uint8_t buffers[2] = {0};
    uint8_t buffer_count = 0;
    
    if (status & STAT_RX0IF) buffers[buffer_count++] = 0;
    if (status & STAT_RX1IF) buffers[buffer_count++] = 1;
    
    for (int i = 0; i < buffer_count && *received_count < max_messages; i++) {
        uint8_t reg_base = (buffers[i] == 0) ? RXB0SIDH : RXB1SIDH;
        uint8_t buffer[14];
        mcp2515_read_registers(reg_base, buffer, 14);
        
        // Message extraction (same as before)
        bool ext_id = (buffer[1] & 0x08) != 0;
        uint32_t id = 0;
        if (ext_id) {
            id = ((uint32_t)buffer[0] << 21) |
                 ((uint32_t)(buffer[1] & 0xE0) << 13) |
                 ((uint32_t)(buffer[1] & 0x03) << 16) |
                 ((uint32_t)buffer[2] << 8) |
                 ((uint32_t)buffer[3]);
        } else {
            id = ((uint32_t)buffer[0] << 3) | (buffer[1] >> 5);
        }
        
        messages[*received_count].id = id;
        messages[*received_count].dlc = buffer[4] & 0x0F;
        if (messages[*received_count].dlc > 8) messages[*received_count].dlc = 8;
        
        for (int j = 0; j < messages[*received_count].dlc; j++) {
            messages[*received_count].data[j] = buffer[5 + j];
        }
        
        (*received_count)++;
        
        // Clear interrupt flag
        mcp2515_bit_modify(CANINTF, buffers[i] ? STAT_RX1IF : STAT_RX0IF, 0);
    }
    
    mcp2515_check_and_clear_errors();
    return (*received_count > 0);
}

/**
 * Sends a CAN message using buffer 0
 * 
 * @param msg Pointer to CANMessage structure containing the message to send
 * @return true if message was queued for transmission, false if failed
 */
bool mcp2515_send_message(CANMessage *msg) {
    // Wait for transmit buffer 0 to be empty
    uint8_t timeout = 50; // ~50ms timeout
    while (timeout--) {
        if ((mcp2515_read_register(TXB0CTRL) & 0x08) == 0) {
            break;  // Buffer is free (TXREQ bit is clear)
        }
        sleep_ms(1);
    }
    
    if (timeout == 0) {
        return false; // Timed out waiting for free buffer
    }
    
    // Prepare extended ID format for J1939 (29-bit)
    // Split the ID across the 4 ID registers
    uint8_t sidh = (uint8_t)((msg->id >> 21) & 0xFF);
    uint8_t sidl = (uint8_t)((msg->id >> 13) & 0xE0) | 0x08 | (uint8_t)((msg->id >> 16) & 0x03);
    uint8_t eid8 = (uint8_t)((msg->id >> 8) & 0xFF);
    uint8_t eid0 = (uint8_t)(msg->id & 0xFF);
    
    // Write ID registers
    mcp2515_write_register(TXB0SIDH, sidh);
    mcp2515_write_register(TXB0SIDL, sidl);
    mcp2515_write_register(TXB0EID8, eid8);
    mcp2515_write_register(TXB0EID0, eid0);
    
    // Set data length (0-8 bytes)
    mcp2515_write_register(TXB0DLC, msg->dlc & 0x0F);
    
    // Write data bytes
    for (uint8_t i = 0; i < msg->dlc; i++) {
        mcp2515_write_register(TXB0D0 + i, msg->data[i]);
    }
    
    // Request transmission (RTS command for buffer 0)
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &(uint8_t){MCP2515_RTS | 0x01}, 1);
    cs_deselect();
    
    return true;
}

/**
 * Waits to receive a CAN message with timeout
 * 
 * @param msg Pointer to CANMessage structure to fill with received data
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if a message was received, false if timed out
 */
bool mcp2515_receive_message_timeout(CANMessage *msg, uint32_t timeout_ms) {
    absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);
    
    while (!time_reached(timeout_time)) {
        if (mcp2515_check_receive()) {
            return mcp2515_receive_message(msg);
        }
        sleep_ms(1); // Small delay to prevent CPU hogging
    }
    
    return false; // Timed out without receiving a message
}

