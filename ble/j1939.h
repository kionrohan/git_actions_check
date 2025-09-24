/**
 * @file j1939.h
 * @brief Header file for J1939 protocol implementation
 * 
 * Defines constants, structures, and function prototypes for
 * handling J1939 protocol messages over CAN bus.
 */

 #ifndef J1939_H
 #define J1939_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include "mcp2515.h" // For CANMessage structure definition
 
 /**
  * @brief J1939 Parameter Group Numbers (PGN) definitions
  */
 #define J1939_PGN_EEC1        61444  // Electronic Engine Controller 1
 #define J1939_PGN_DM1         65226  // Diagnostic Message 1 (Active DTCs)
 
 /**
  * @brief J1939 CAN IDs for specific messages
  */
 #define EEC1_ID               0xCF00400 // ID for EEC1 messages

 
 
 /**
  * @brief Structure to hold SPN (Suspect Parameter Number) information
  */
 typedef struct {
     uint32_t spn;             // SPN number
     uint8_t start_position;   // Starting byte in the message
     uint8_t length;           // Length in bytes
     float resolution;         // Scaling factor
     float offset;             // Offset value
     char message[50];         // Description of the parameter
 } SPNInfo;
 
 /**
  * @brief Decode a SPN value from raw data
  * 
  * @param data Raw message data
  * @param start_position Starting byte position
  * @param length Length in bytes
  * @param resolution Scaling factor
  * @param offset Offset value
  * @return float Decoded physical value
  */
 uint32_t extract_raw_spn_value(const uint8_t *data, uint8_t start_position, uint8_t length);
 void extract_multiple_raw_values(const CANMessage *msg, 
    uint32_t *raw_values,
    const uint8_t *positions, 
    const uint8_t *lengths, 
    uint8_t num_values);
 
 /**
  * @brief Receive and process a J1939 CAN message
  * 
  * @param msg Pointer to store the received CAN message
  * @return bool True if message received, false otherwise
  */
 bool j1939_receive_message(CANMessage *msg);
 
 
 /**
  * @brief Format CAN data for application use
  * 
  * @param msg Pointer to the CAN message to format
  * @return char* Formatted string containing the message data
  */
 char* format_can_data_for_app(const CANMessage *msg);
 
 
 
 #endif // J1939_H