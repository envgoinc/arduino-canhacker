/*
 * CanHackerMcp2515.cpp
 *
 *  Created on: 17 ���. 2015 �.
 *      Author: Dmitry
 */
#include <Arduino.h>
#include <stdio.h>
#include <ctype.h>
#include "CanHackerMcp2515.h"

CanHackerMcp2515::CanHackerMcp2515(uint8_t cs, const uint32_t spi_clock) {

    _mcp2515 = new MCP2515(_cs, spi_clock);
}

CanHackerMcp2515::~CanHackerMcp2515() {
    delete _mcp2515;
}

CanHackerMcp2515::begin() {
    MCP2515::ERROR error;

    error = mcp2515->reset();
    if(error != MCP2515::ERROR_OK) {
        return ERROR_MCP2515_INIT;
    }
    error = mcp2515->setConfigMode();
    if( error != MCP2515::ERROR_OK) {
        return ERROR_MCP2515_INIT_CONFIG;
    }
    error = mcp2515->setClkOut(CLKOUT_DIV1);
    if( error != MCP2515::ERROR_OK) {
        return ERROR_MCP2515_INIT;
    }
}

CanHackerMcp2515::ERROR CanHackerMcp2515::setBitrate() {
    MCP2515::ERROR error = mcp2515->setBitrate(bitrate, canClock);
    if (error != MCP2515::ERROR_OK) {
        return ERROR_MCP2515_INIT_BITRATE;
    }
}

CanHackerMcp2515::ERROR CanHackerMcp2515::setMode(InterfaceMode_e mode) {
    MCP2515::ERROR error;
    switch(mode) {
        case MODE_NORMAL:
        error = mcp2515->setNormalMode();
        break;
        case MODE_LOOPBACK:
        error = mcp2515->setLoopbackMode();
        break;
        case MODE_LISTEN_ONLY:
        error = mcp2515->setListenOnlyMode();
        break;
        default:
        error = ERROR_FAIL;
        break;
    }
    if(error != MCP2515::ERROR_OK) {
        return ERROR_INTERFACE_INVALID_MODE;
    }
}

CanHackerMcp2515::ERROR CanHackerMcp2515::disconnectCan() {
    _isConnected = false;
    mcp2515->setConfigMode();
    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::writeCan(const struct can_frame *frame) {
    if (mcp2515->sendMessage(frame) != MCP2515::ERROR_OK) {
        return ERROR_MCP2515_SEND;
    }

    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::pollReceiveCan() {
    if (!isConnected()) {
        return ERROR_OK;
    }

    while (mcp2515->checkReceive()) {
        struct can_frame frame;
        if (mcp2515->readMessage(&frame) != MCP2515::ERROR_OK) {
            return ERROR_MCP2515_READ;
        }

        ERROR error = receiveCanFrame(&frame);
        if (error != ERROR_OK) {
            return error;
        }
    }

    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::receiveCan(const MCP2515::RXBn rxBuffer) {
    if (!isConnected()) {
        return ERROR_OK;
    }

    struct can_frame frame;
    MCP2515::ERROR result = mcp2515->readMessage(rxBuffer, &frame);
    if (result == MCP2515::ERROR_NOMSG) {
        return ERROR_OK;
    }

    if (result != MCP2515::ERROR_OK) {
        return ERROR_MCP2515_READ;
    }

    return receiveCanFrame(&frame);
}

MCP2515 *CanHackerMcp2515::getMcp2515() {
    return mcp2515;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::processInterrupt() {
    bool msg_received = false;

    if (!isConnected()) {
        return ERROR_OK;
        writeDebugStream(F("Process interrupt while not connected\n"));
        return ERROR_NOT_CONNECTED;
    }

    uint8_t irq = mcp2515->getInterrupts();

    if (irq & MCP2515::CANINTF_ERRIF) {
        _debugStream->print(F("ERRIF\r\n"));
        processError();
        mcp2515->clearERRIF();
    }

    if (irq & MCP2515::CANINTF_RX0IF) {
        ERROR error = receiveCan(MCP2515::RXB0);
        if (error != ERROR_OK) {
            return error;
        } else {
            msg_received = true;
        }
    }

    if (irq & MCP2515::CANINTF_RX1IF) {
        ERROR error = receiveCan(MCP2515::RXB1);
        if (error != ERROR_OK) {
            return error;
        } else {
            msg_received = true;
        }
    }

    if (msg_received) {
        checkErrorCounter();
    }

    /*if (irq & (MCP2515::CANINTF_TX0IF | MCP2515::CANINTF_TX1IF | MCP2515::CANINTF_TX2IF)) {
        _debugStream->print("MCP_TXxIF\r\n");
        //stopAndBlink(1);
    }*/


    if (irq & MCP2515::CANINTF_WAKIF) {
        _debugStream->print(F("MCP_WAKIF\r\n"));
        mcp2515->clearWAKIF();
    }

    if (irq & MCP2515::CANINTF_MERRF) {
        _debugStream->print(F("MERRF\r\n"));
        mcp2515->clearMERR();
    }

    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::checkErrorCounter() {
    char out[15];
    ERROR error;
    const uint8_t errorCount = mcp2515->errorCountRX();

    // check for an increase in receive error count.  If there is, send
    // error frame.
    if(_rxErrorCount != errorCount) {
        _rxErrorCount = errorCount;
        error = createErrorStatus('s', out, sizeof(out));
        if (error != ERROR_OK) {
            return error;
        }
        error = writeStream(out);
        if (error != ERROR_OK) {
            return error;
        }
    }
    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::processError() {
    uint8_t errFlags = mcp2515->getErrorFlags();
    char out[15];
    ERROR error;
    CAN_STATE state;
    char state_char;

    if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
        mcp2515->clearRXnOVRFlags();
        error = createErrorStatus('o', out, sizeof(out));
        if (error != ERROR_OK) {
            return error;
        }
        error = writeStream(out);
        if (error != ERROR_OK) {
            return error;
        }
    }

    state = mcp2515->getBusState();
    if (state != _state) {
        _state = state;
        switch (state) {
            case BUS_ACTIVE:
                state_char = 'a';
                break;
            case BUS_PASSIVE:
                state_char = 'p';
                break;
            case BUS_WARN:
                state_char = 'w';
                break;
            case BUS_OFF:
                state_char = 'f';
                break;
            default:
                break;
        }
        error = createBusState(state_char, out, sizeof(out));
        if (error != ERROR_OK) {
            return error;
        }
        error = writeStream(out);
        if (error != ERROR_OK) {
            return error;
        }
    }
    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::setFilter(const uint32_t filter) {
    if (isConnected()) {
        writeDebugStream(F("Filter cannot be set while connected\n"));
        return ERROR_CONNECTED;
    }

    MCP2515::RXF filters[] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2, MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
    for (int i=0; i<6; i++) {
        MCP2515::ERROR result = mcp2515->setFilter(filters[i], false, filter);
        if (result != MCP2515::ERROR_OK) {
            return ERROR_MCP2515_FILTER;
        }
    }

    return ERROR_OK;
}

CanHackerMcp2515::ERROR CanHackerMcp2515::setFilterMask(const uint32_t mask) {
    if (isConnected()) {
        writeDebugStream(F("Filter mask cannot be set while connected\n"));
        return ERROR_CONNECTED;
    }

    MCP2515::MASK masks[] = {MCP2515::MASK0, MCP2515::MASK1};
    for (int i=0; i<2; i++) {
        MCP2515::ERROR result = mcp2515->setFilterMask(masks[i], false, mask);
        if (result != MCP2515::ERROR_OK) {
            return ERROR_MCP2515_FILTER;
        }
    }

    return ERROR_OK;
}

uint8_t CanHackerMcp2515::getRxErrorCount(void)
{
    return mcp2515->errorCountRX();
}

uint8_t CanHackerMcp2515::getTxErrorCount(void)
{
    return mcp2515->errorCountTX();
}
