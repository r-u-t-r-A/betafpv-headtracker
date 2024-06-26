
 
 // Basic setup
#define CRSF_MAX_CHANNEL 16
//#define CRSF_FRAME_SIZE_MAX 64
//#define SERIAL_BAUDRATE 420000 //low baud for Arduino Nano , the TX module will auto detect baud. max packet rate is 250Hz.

 // Device address & type
#define RADIO_ADDRESS                  0xEA
#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS                  0x16

// Define RC input limite
#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811

//Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define ELRS_LUA_COMMAND_REQUEST 0
#define ELRS_LUA_COMMAND_PACKET_RATE 1
#define ELRS_LUA_COMMAND_TELEM_RATIO 2
#define ELRS_LUA_COMMAND_SWITCH_MODE 3
#define ELRS_LUA_COMMAND_MODEL_MATCH 4
#define ELRS_LUA_COMMAND_TX_POWER_SET 5
#define ELRS_LUA_COMMAND_MAX_TX_POWER 6
#define ELRS_LUA_COMMAND_DYNAMIC_POWER 7
#define ELRS_LUA_COMMAND_VTX_ADMIN 8
#define ELRS_LUA_COMMAND_VTX_BAND 9
#define ELRS_LUA_COMMAND_VTX_CH 10 //ok
#define ELRS_LUA_COMMAND_VTX_PWR_LVL 11 //ok
#define ELRS_LUA_COMMAND_VTX_PIT_MODE 12
#define ELRS_LUA_COMMAND_SEND_VTX 13
//#define ELRS_LUA_COMMAND_WIFI_CONN 14
#define ELRS_LUA_COMMAND_ENABLE_WIFI 15 //ok
#define ELRS_LUA_COMMAND_EN_RX_WIFI 16  //ok
#define ELRS_LUA_COMMAND_BIND 17

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
//#define CRSF_TIME_BETWEEN_FRAMES_US     4000 // 4 ms 250Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX_RC   60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_FRAME_LENGTH 24;   // length of type + payload + crc


#define CRSF_CMD_PACKET_SIZE  8

// ELRS command
#define ELRS_ADDRESS                   0xEE
#define ELRS_BIND_COMMAND              0xFF
#define ELRS_WIFI_COMMAND              0xFE
#define ELRS_PKT_RATE_COMMAND          1
#define ELRS_TLM_RATIO_COMMAND         2
#define ELRS_POWER_COMMAND             3
#define TYPE_SETTINGS_WRITE            0x2D
#define ADDR_RADIO                     0xEA  //  Radio Transmitter

//#include <Arduino.h>
#include "crsf_protocol.h"
//#pragma once

#define CRSF_MAX_PARAMS 55 // one extra required, max observed is 47 in Diversity Nano RX
#define CRSF_MAX_DEVICES 4
#define CRSF_MAX_NAME_LEN 16
#define CRSF_MAX_STRING_BYTES 2500 // max observed is 2010 in Nano RX
#define CRSF_STRING_BYTES_AVAIL(current) (CRSF_MAX_STRING_BYTES - ((char *)(current)-mp->strings))

typedef struct
{
    uint8_t address;
    uint8_t number_of_params;
    uint8_t params_version;
    uint32_t serial_number;
    uint32_t hardware_id;
    uint32_t firmware_id;
    char name[CRSF_MAX_NAME_LEN];
} crsf_device_t;

typedef enum
{
    MODULE_UNKNOWN,
    MODULE_ELRS,
    MODULE_OTHER,
} module_type_t;

uint8_t protocol_module_is_elrs();

#define CRSF_MAX_CHANNEL 16

//extern int rcChannels[CRSF_MAX_CHANNEL];

#define CRSF_MAX_CHUNK_SIZE 58 // 64 - header - type - destination - origin
#define CRSF_MAX_CHUNKS 5      // not in specification. Max observed is 3 for Nano RX

extern module_type_t module_type;
extern uint8_t device_idx; // current device index

extern char recv_param_buffer[];
extern char *recv_param_ptr;

// Basic setup
#ifdef DEBUG
#define SERIAL_BAUDRATE 115200 // low baud for Arduino Nano , the TX module will auto detect baud. max packet rate is 250Hz.
#else
#define SERIAL_BAUDRATE 400000 // testing 3750000//1870000
#endif
// Device address & type
#define RADIO_ADDRESS 0xEA
#define ADDR_MODULE 0xEE //  Crossfire transmitter
#define TYPE_CHANNELS 0x16

// Define RC input limite
#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811

// Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_FRAME_PERIOD_MIN 850   // 1000Hz 1ms, but allow shorter for offset cancellation
#define CRSF_FRAME_PERIOD_MAX 50000 // 25Hz  40ms, but allow longer for offset cancellation
// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US 1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US 5000   // 4 ms 250Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE 26
#define CRSF_FRAME_LENGTH 24 // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE 8
#define LinkStatisticsFrameLength 10 //

// ELRS command
#define ELRS_ADDRESS 0xEE
#define ELRS_RX_ADDRESS 0xEC
#define ELRS_BIND_COMMAND 0xFF
#define ELRS_WIFI_COMMAND 0xFE
#define ELRS_PKT_RATE_COMMAND 1
#define ELRS_TLM_RATIO_COMMAND 2
#define ELRS_POWER_COMMAND 3

#define ADDR_RADIO 0xEA //  Radio Transmitter

// Frame Type
#define TYPE_GPS 0x02
#define TYPE_VARIO 0x07
#define TYPE_BATTERY 0x08
#define TYPE_HEARTBEAT 0x0b
#define TYPE_VTX 0x0F
#define TYPE_VTX_TELEM 0x10
#define TYPE_LINK 0x14
#define TYPE_CHANNELS 0x16
#define TYPE_RX_ID 0x1C
#define TYPE_TX_ID 0x1D
#define TYPE_ATTITUDE 0x1E
#define TYPE_FLIGHT_MODE 0x21
#define TYPE_PING_DEVICES 0x28
#define TYPE_DEVICE_INFO 0x29
#define TYPE_REQUEST_SETTINGS 0x2A
#define TYPE_SETTINGS_ENTRY 0x2B
#define TYPE_SETTINGS_READ 0x2C
#define TYPE_SETTINGS_WRITE 0x2D
#define TYPE_ELRS_INFO 0x2E
#define TYPE_COMMAND_ID 0x32
#define TYPE_RADIO_ID 0x3A

// Frame Subtype
#define UART_SYNC 0xC8
#define CRSF_SUBCOMMAND 0x10
#define COMMAND_MODEL_SELECT_ID 0x05

#define TELEMETRY_RX_PACKET_SIZE 64

#define CRSF_CRC_POLY 0xd5

#define CRSF_MAX_PACKET_LEN 64

#define SEND_MSG_BUF_SIZE 64 // don't send more than one chunk
#define ADDR_BROADCAST 0x00  //  Broadcast address

#define MODULE_IS_ELRS (module_type == MODULE_ELRS)
#define MODULE_IS_UNKNOWN (module_type == MODULE_UNKNOWN)

typedef struct
{
    uint8_t update;
    uint8_t bad_pkts;
    uint16_t good_pkts;
    uint8_t flags;
    char flag_info[CRSF_MAX_NAME_LEN];
} elrs_info_t;

/// UART Handling ///
static volatile uint8_t SerialInPacketLen; // length of the CRSF packet as measured
static volatile uint8_t SerialInPacketPtr; // index where we are reading/writing
static volatile bool CRSFframeActive;      // = false; //since we get a copy of the serial data use this flag to know when to ignore it


