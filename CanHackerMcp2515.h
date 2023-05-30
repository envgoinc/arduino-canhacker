/*
 * CanHacker.h
 *
 *      Author: Dmitry
 */

#pragma once

#include <can.h>
#include <mcp2515.h>

#define CAN_MIN_DLEN 1
#define HEX_PER_BYTE 2
#define MIN_MESSAGE_DATA_HEX_LENGTH CAN_MIN_DLEN * HEX_PER_BYTE
#define MAX_MESSAGE_DATA_HEX_LENGTH CAN_MAX_DLEN * HEX_PER_BYTE
#define MIN_MESSAGE_LENGTH 5

#define CANHACKER_CMD_MAX_LENGTH 26

#define CANHACKER_SERIAL_RESPONSE     "N0001\r"
#define CANHACKER_SW_VERSION_RESPONSE "v0107\r"
#define CANHACKER_VERSION_RESPONSE    "V1010\r"

class CanHackerMcp2515 {
    public:
        enum ERROR {
            ERROR_OK,
            ERROR_MCP2515_INIT,
            ERROR_MCP2515_INIT_CONFIG,
            ERROR_MCP2515_INIT_BITRATE,
            ERROR_MCP2515_INIT_SET_MODE,
            ERROR_MCP2515_SEND,
            ERROR_MCP2515_READ,
            ERROR_MCP2515_FILTER,
            ERROR_MCP2515_ERRIF,
            ERROR_MCP2515_MERRF
        };

        typedef enum {
            MODE_NORMAL = 0,
            MODE_LOOPBACK,
            MODE_LISTEN_ONLY
        } InterfaceMode_e;

        CanHackerMcp2515(uint8_t cs, const uint32_t spi_clock = 0);
        ~CanHackerMcp2515();
        ERROR setMode();
        ERROR pollReceiveCan();
        ERROR receiveCan(const MCP2515::RXBn rxBuffer);
        MCP2515 *getMcp2515();
        ERROR processInterrupt();

    private:
        CAN_CLOCK canClock = MCP_16MHZ;
        bool _timestampEnabled = false;
        bool _listenOnly = false;
        bool _loopback = false;
        uint8_t _cs;
        MCP2515 *_mcp2515;
        CAN_SPEED bitrate;
        bool _isConnected = false;
        uint8_t _rxErrorCount;
        CAN_STATE _state;
        uint8_t scratch = 0;

        ERROR checkErrorCounter();

        ERROR processError();

        ERROR setFilter(const uint32_t filter);
        ERROR setFilterMask(const uint32_t mask);

        ERROR connectCan();
        ERROR disconnectCan();
        ERROR writeCan(const struct can_frame *);

};
