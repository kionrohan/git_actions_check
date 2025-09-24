// mcp2515.h - Header file for MCP2515 CAN Bus Controller Driver
// Defines registers, commands, and function prototypes for interfacing with MCP2515
// Configured for Raspberry Pi Pico using SPI communication

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

// SPI configuration for MCP2515
// Pin assignments for interfacing Raspberry Pi Pico with MCP2515
#define MCP2515_SPI_PORT spi0      // Using SPI0 peripheral
#define PIN_MISO 4                 // GPIO4 - Master In Slave Out
#define PIN_CS   5                 // GPIO5 - Chip Select (active low)
#define PIN_SCK  6                 // GPIO6 - Serial Clock
#define PIN_MOSI 7                 // GPIO7 - Master Out Slave In
#define PIN_INT  21                // GPIO21 - Interrupt from MCP2515 (active low)

// MCP2515 SPI Command Bytes
#define MCP2515_RESET     0xC0     // Software reset command
#define MCP2515_READ      0x03     // Read from register
#define MCP2515_WRITE     0x02     // Write to register
#define MCP2515_RTS       0x80     // Request to send (start transmission)
#define MCP2515_STATUS    0xA0     // Read status register
#define MCP2515_BIT_MODIFY 0x05    // Modify specific bits in a register

// MCP2515 Register Addresses - Control and Configuration
// #define CANCTRL     0x0F    // CAN control register - sets operating mode
// #define CANSTAT     0x0E    // CAN status register - reflects current operating mode
// #define CNF1        0x2A    // Configuration register 1 - bit timing
// #define CNF2        0x29    // Configuration register 2 - bit timing
// #define CNF3        0x28    // Configuration register 3 - bit timing
// #define CANINTE     0x2B    // Interrupt enable register
// #define CANINTF     0x2C    // Interrupt flag register
// #define EFLG        0x2D

#define CANCTRL  0x0F
#define CANSTAT  0x0E
#define CNF1     0x2A
#define CNF2     0x29
#define CNF3     0x28
#define CANINTE  0x2B
#define CANINTF  0x2C
#define EFLG     0x2D
#define TEC      0x1C
#define REC      0x1D
#define CIOCON   0x0F

// Transmit buffer registers - Buffer 0
#define TXB0CTRL    0x30    // Transmit buffer 0 control register
#define TXB0SIDH    0x31    // Transmit buffer 0 standard ID high
#define TXB0SIDL    0x32    // Transmit buffer 0 standard ID low
#define TXB0EID8    0x33    // Transmit buffer 0 extended ID high
#define TXB0EID0    0x34    // Transmit buffer 0 extended ID low
#define TXB0DLC     0x35    // Transmit buffer 0 data length code
#define TXB0D0      0x36    // Transmit buffer 0 data byte 0 (first of 8 possible data bytes)

// Receive buffer 0 registers
#define RXB0CTRL    0x60    // Receive buffer 0 control register
#define RXB0SIDH    0x61    // Receive buffer 0 standard ID high
#define RXB0SIDL    0x62    // Receive buffer 0 standard ID low
#define RXB0EID8    0x63    // Receive buffer 0 extended ID high
#define RXB0EID0    0x64    // Receive buffer 0 extended ID low
#define RXB0DLC     0x65    // Receive buffer 0 data length code
#define RXB0D0      0x66    // Receive buffer 0 data byte 0 (first of 8 possible data bytes)

// Receive buffer 1 registers
#define RXB1CTRL    0x70    // Receive buffer 1 control register
#define RXB1SIDH    0x71    // Receive buffer 1 standard ID high
#define RXB1SIDL    0x72    // Receive buffer 1 standard ID low
#define RXB1EID8    0x73    // Receive buffer 1 extended ID high
#define RXB1EID0    0x74    // Receive buffer 1 extended ID low
#define RXB1DLC     0x75    // Receive buffer 1 data length code
#define RXB1D0      0x76    // Receive buffer 1 data byte 0 (first of 8 possible data bytes)

// MCP2515 operation modes - set in CANCTRL register
#define MODE_NORMAL    0x00    // Normal operation mode - sends and receives messages
#define MODE_SLEEP     0x20    // Sleep mode for low power consumption
#define MODE_LOOPBACK  0x40    // Loopback mode for testing without actual CAN bus
#define MODE_LISTENONLY 0x60   // Listen-only mode - receive only, no transmission
#define MODE_CONFIG    0x80    // Configuration mode - must be in this mode to change settings
#define MODE_MASK      0xE0    // Mask for extracting mode bits from register

// Status bits for checking message reception
#define STAT_RX0IF     0x01    // Message received in buffer 0
#define STAT_RX1IF     0x02    // Message received in buffer 1
#define STAT_TX0REQ         0x04
#define STAT_TX1REQ         0x08
#define STAT_TX2REQ         0x10

/**
 * CAN Message Structure
 * Used for both transmitting and receiving CAN messages
 */
typedef struct {
    uint32_t id;            // CAN identifier (11-bit standard or 29-bit extended)
    uint8_t dlc;            // Data Length Code (0-8 bytes)
    uint8_t data[8];        // Data bytes (up to 8)
    bool is_error_frame;    // Flag for error frames
} CANMessage;

// Function declarations

/**
 * Activates the chip select line for SPI communication
 */
void cs_select(void);

/**
 * Deactivates the chip select line ending SPI communication
 */
void cs_deselect(void);

/**
 * Writes a value to a specific MCP2515 register
 * 
 * @param address Register address to write to
 * @param value Value to write
 */
void mcp2515_write_register(uint8_t address, uint8_t value);

/**
 * Reads a value from a specific MCP2515 register
 * 
 * @param address Register address to read from
 * @return Value read from the register
 */
uint8_t mcp2515_read_register(uint8_t address);

/**
 * Reads multiple consecutive registers from MCP2515
 * 
 * @param address Starting register address
 * @param values Buffer to store read values
 * @param count Number of registers to read
 */
void mcp2515_read_registers(uint8_t address, uint8_t *values, uint8_t count);

/**
 * Modifies specific bits in a register without affecting others
 * 
 * @param address Register address to modify
 * @param mask Bit mask (1s for bits to modify)
 * @param data New values for the masked bits
 */
void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data);

/**
 * Gets current status of the MCP2515
 * 
 * @return Status byte with interrupt flags
 */
uint8_t mcp2515_get_status(void);

/**
 * Sets MCP2515 operating mode
 * 
 * @param mode Mode to set (normal, config, sleep, etc.)
 * @return true if successful, false if timed out
 */
bool mcp2515_set_mode(uint8_t mode);

/**
 * Initializes the MCP2515 for J1939 communication
 * 
 * @return true if initialization successful, false otherwise
 */
bool mcp2515_init(void);

/**
 * Checks if any messages are available for reception
 * 
 * @return true if message available, false otherwise
 */
bool mcp2515_check_receive(void);

/**
 * Receives a CAN message if available
 * 
 * @param msg Pointer to CANMessage structure to fill
 * @return true if message received, false otherwise
 */
bool mcp2515_receive_message(CANMessage *msg);

/**
 * Sends a CAN message
 * 
 * @param msg Pointer to CANMessage structure with message to send
 * @return true if message queued for transmission, false if failed
 */
bool mcp2515_send_message(CANMessage *msg);

/**
 * Waits to receive a CAN message with timeout
 * 
 * @param msg Pointer to CANMessage structure to fill
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if message received, false if timed out
 */
bool mcp2515_receive_message_timeout(CANMessage *msg, uint32_t timeout_ms);



#endif // MCP2515_H