/**
 * @file j1939.c
 * @brief Implementation of J1939 protocol functions for CAN communication
 * 
 * This file contains functions for decoding, receiving and processing
 * J1939 CAN messages, including diagnostic trouble codes (DTCs).
 */

 #include <stdio.h>
 #include <string.h>
 #include "j1939.h"
 #include "mcp2515.h"
 #include "pico/time.h"
 
 /**
  * @brief Receive and process a J1939 CAN message
  * 
  * @param msg Pointer to store the received CAN message
  * @return bool True if a message was received, false otherwise
  */
 bool j1939_receive_message(CANMessage *msg) {
     // Receive message from MCP2515 CAN controller
     if (!mcp2515_receive_message(msg)) {
         return false; // No message received
     }
     
     // Extract J1939 parameters
     uint8_t priority = (msg->id >> 26) & 0x7;
     uint32_t pgn = (msg->id >> 8) & 0x3FFFF;
     uint8_t source_address = msg->id & 0xFF;
     
     // Print message information
    //  printf("Received CAN message, ID: 0x%08X, DLC: %d\n", msg->id, msg->dlc);
    //  printf("Data: ");
    //  for (int i = 0; i < msg->dlc; i++) {
    //      printf("%02X ", msg->data[i]);
    //  }
    //  printf("\n");
     
     return true;
 }
 
 
/**
 * @brief Format CAN data for application use with raw values only
 * 
 * Creates a formatted string representation of CAN message data
 * including PGN, SPN, FMI, and raw values for display
 * or transmission to other application layers.
 * 
 * @param msg Pointer to the CAN message to format
 * @return char* Formatted string containing the message data
 */
char* format_can_data_for_app(const CANMessage *msg) {
    static char response[256];
    int offset = 0;

    // Extract PGN (Parameter Group Number)
    uint32_t pgn = (msg->id >> 8) & 0x3FFFF;
    offset += snprintf(response + offset, sizeof(response) - offset,
                      "PGN: %u\n", pgn);

    // Generic extraction of SPN and FMI from all J1939 messages
    if (msg->dlc >= 5) {  // Need at least 5 bytes to extract SPN and FMI
        // Extract SPN (Suspect Parameter Number) using the formula provided
        uint32_t spn = ((uint32_t)msg->data[2]) | 
                       ((uint32_t)msg->data[3] << 8) | 
                       (((uint32_t)(msg->data[4] & 0xE0)) << 11);
        
        // Extract FMI (Failure Mode Identifier)
        uint8_t fmi = msg->data[4] & 0x1F;
        
        offset += snprintf(response + offset, sizeof(response) - offset,
                         "SPN: %u\nFMI: %u\n", spn, fmi);
        
        uint32_t raw_value = ((uint32_t)msg->data[0] << 8) | msg->data[1];
        offset += snprintf(response + offset, sizeof(response) - offset,
                         "Value: %u\n", raw_value);
    } 
    
    return response;
}