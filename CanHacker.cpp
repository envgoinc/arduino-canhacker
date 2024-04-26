/*
 * CanHacker.cpp
 *
 *  Created on: 17 ���. 2015 �.
 *      Author: Dmitry
 */
#include <Arduino.h>
#include <stdio.h>
#include <ctype.h>

#include "CanHacker.h"
#include "lib.h"

const char hex_asc_upper[] = "0123456789ABCDEF";

#define hex_asc_upper_lo(x)    hex_asc_upper[((x) & 0x0F)]
#define hex_asc_upper_hi(x)    hex_asc_upper[((x) & 0xF0) >> 4]

static inline void put_hex_byte(char *buf, __u8 byte)
{
    buf[0] = hex_asc_upper_hi(byte);
    buf[1] = hex_asc_upper_lo(byte);
}

static inline void _put_id(char *buf, int end_offset, canid_t id)
{
    /* build 3 (SFF) or 8 (EFF) digit CAN identifier */
    while (end_offset >= 0) {
        buf[end_offset--] = hex_asc_upper[id & 0xF];
        id >>= 4;
    }
}

#define put_sff_id(buf, id) _put_id(buf, 2, id)
#define put_eff_id(buf, id) _put_id(buf, 7, id)

CanHacker::CanHacker(Stream *stream, Stream *debugStream, uint8_t cs, const uint32_t spi_clock) {
    _stream = stream;
    _debugStream = debugStream;

    writeDebugStream(F("Initialization\n"));

    #ifdef MCP_DRIVER
    if(spi_clock == 0) {
      can_instance = new MCP_CAN(cs);
    } else {
      can_instance = new MCP_CAN(cs, spi_clock);
    }
    #endif /* MCP_DRIVER */

    #ifdef STM_DRIVER
    can_instance = new STM_CAN(CAN2, STM_CAN::ALT);
    #endif /* STM_DRIVER */


    can_instance->begin();

}

CanHacker::~CanHacker() {
    delete can_instance;
}

Stream *CanHacker::getInterfaceStream() {
    return _stream;
}

BASE_CAN *CanHacker::get_can_instance(){
  return can_instance;
}

void CanHacker::setClock(BASE_CAN::CAN_CLOCK clock){
    can_instance->setClock(clock);
}

CanHacker::ERROR CanHacker::connectCan() {
    can_instance->connectCan();

    ERROR error = (ERROR)can_instance->getError();

    if(error == ERROR_BITRATE){
        writeDebugStream(F("setBitrate error:\n"));
        writeDebugStream((int)error);
        writeDebugStream("\n");
        return ERROR_BITRATE;
    }
    else if (error == ERROR_SET_MODE) {
        writeDebugStream(F("error setting mode: "));
        writeDebugStream((int)error);
        writeDebugStream("\n");
        return ERROR_SET_MODE;
    }
    else if(error != ERROR_OK){
        writeDebugStream(F("error: "));
        writeDebugStream((int)error);
        writeDebugStream("\n");
        return ERROR_INVALID_COMMAND;
    }

    return ERROR_OK;
}

CanHacker::ERROR CanHacker::disconnectCan() {
    return (ERROR)can_instance->disconnectCan();
}

bool CanHacker::isConnected() {
    return can_instance->isConnected();
}

CanHacker::ERROR CanHacker::writeCan(const can_message &frame) {
    if (can_instance->write(frame)) {
        return ERROR_OK;
    }

    return ERROR_SEND;
}

uint16_t CanHacker::getTimestamp() {
    return millis() % TIMESTAMP_LIMIT;
}

CanHacker::ERROR CanHacker::receiveSetBitrateCommand(const char *buffer, const int length) {
    if (isConnected()) {
        writeDebugStream(F("Bitrate command cannot be called while connected\n"));
        writeStream(BEL);
        return ERROR_CONNECTED;
    }

    if (length < 2) {
        writeStream(BEL);
        writeDebugStream(F("Bitrate command must by 2 bytes long\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }
    switch(buffer[1]) {
        case '0':
            writeDebugStream(F("Set bitrate 10KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_10KBPS);
            break;
        case '1':
            writeDebugStream(F("Set bitrate 20KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_20KBPS);
            break;
        case '2':
            writeDebugStream(F("Set bitrate 50KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_50KBPS);
            break;
        case '3':
            writeDebugStream(F("Set bitrate 100KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_100KBPS);
            break;
        case '4':
            writeDebugStream(F("Set bitrate 125KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_125KBPS);
            break;
        case '5':
            writeDebugStream(F("Set bitrate 250KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_250KBPS);
            break;
        case '6':
            writeDebugStream(F("Set bitrate 500KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_500KBPS);
            break;
        case '7':
            writeDebugStream(F("Bitrate 7 is not supported\n"));
            writeDebugStream((const uint8_t*)buffer, length);
            writeDebugStream('\n');
            writeStream(BEL);
            return ERROR_INVALID_COMMAND;
            break;
        case '8':
            writeDebugStream(F("Set bitrate 1000KBPS\n"));
            can_instance->setBaudRate(BASE_CAN::CAN_1000KBPS);
            break;
        default:
            writeDebugStream(F("Unexpected bitrate\n"));
            writeDebugStream((const uint8_t*)buffer, length);
            writeDebugStream('\n');
            writeStream(BEL);
            return ERROR_INVALID_COMMAND;
            break;
    }

    return writeStream(CR);
}

CanHacker::ERROR CanHacker::writeStream(const char character) {
    char str[2];
    str[0] = character;
    str[1] = '\0';
    return writeStream(str);
}

CanHacker::ERROR CanHacker::writeStream(const char *buffer) {
    /*if (_stream->availableForWrite() < strlen(buffer)) {
        return ERROR_SERIAL_TX_OVERRUN;
    }*/
    size_t printed = _stream->print(buffer);
    if (printed != strlen(buffer)) {
        return ERROR_SERIAL_TX_OVERRUN;
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::writeDebugStream(const char character) {
    if (_debugStream != NULL) {
        _debugStream->write(character);
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::writeDebugStream(const char *buffer) {
    if (_debugStream != NULL) {
        _debugStream->print(buffer);
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::writeDebugStream(const __FlashStringHelper *ifsh) {
    if (_debugStream != NULL) {
        _debugStream->print(ifsh);
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::writeDebugStream(const uint8_t *buffer, size_t size) {
    if (_debugStream != NULL) {
        _debugStream->write(buffer, size);
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::writeDebugStream(const int buffer) {
    if (_debugStream != NULL) {
        _debugStream->print(buffer);
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::receiveCommand(const char *buffer, const int length) {
    switch (buffer[0]) {
        case COMMAND_GET_SERIAL: {
            return writeStream(CANHACKER_SERIAL_RESPONSE);
        }

        case COMMAND_GET_SW_VERSION: {
            return writeStream(CANHACKER_SW_VERSION_RESPONSE);
        }

        case COMMAND_GET_VERSION: {
            return writeStream(CANHACKER_VERSION_RESPONSE);
        }

        case COMMAND_SEND_11BIT_ID:
        case COMMAND_SEND_29BIT_ID:
        case COMMAND_SEND_R11BIT_ID:
        case COMMAND_SEND_R29BIT_ID:
            return receiveTransmitCommand(buffer, length);

        case COMMAND_CLOSE_CAN_CHAN:
            return receiveCloseCommand(buffer, length);

        case COMMAND_OPEN_CAN_CHAN:
            return receiveOpenCommand(buffer, length);

        case COMMAND_SET_BITRATE:
            return receiveSetBitrateCommand(buffer, length);

        case COMMAND_SET_ACR:
            return receiveSetAcrCommand(buffer, length);

        case COMMAND_SET_AMR:
            return receiveSetAmrCommand(buffer, length);

        case COMMAND_SET_BTR: {
            if (isConnected()) {
                writeStream(BEL);
                writeDebugStream(F("SET_BTR command cannot be called while connected\n"));
                return ERROR_CONNECTED;
            }
            writeDebugStream(F("SET_BTR not supported\n"));
            return writeStream(CR);
        }

        case COMMAND_LISTEN_ONLY:
            return receiveListenOnlyCommand(buffer, length);

        case COMMAND_TIME_STAMP:
            return receiveTimestampCommand(buffer, length);

        case COMMAND_WRITE_REG:
        case COMMAND_READ_REG: {
            return writeStream(CR);
        }

        case COMMAND_READ_STATUS:
        case COMMAND_READ_ECR:
        case COMMAND_READ_ALCR: {
            if (!isConnected()) {
                writeDebugStream(F("Read status, ecr, alcr while not connected\n"));
                writeStream(BEL);
                return ERROR_NOT_CONNECTED;
            }
            return writeStream(CR);
        }

        default: {
            writeStream(BEL);
            writeDebugStream(F("Unknown command received\n"));
            writeDebugStream((const uint8_t*)buffer, length);
            writeDebugStream('\n');
            return ERROR_UNKNOWN_COMMAND;
        }
    }
}

CanHacker::ERROR CanHacker::receiveCanFrame(const can_message *frame) {
    char out[35];

    ERROR error = createTransmit(frame, out, sizeof(out));
    if (error != ERROR_OK) {
        return error;
    }
    error = writeStream(out);
    if (error != ERROR_OK) {
        return error;
    }
    return ERROR_OK;
}

CanHacker::ERROR CanHacker::parseTransmit(const char *buffer, int length, can_message *frame) {
    if (length < MIN_MESSAGE_LENGTH) {
        writeDebugStream(F("Transmit message lenght < minimum\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }

    int isExended = 0;
    int isRTR = 0;

    switch (buffer[0]) {
        case 't':
            break;
        case 'T':
            isExended = 1;
            break;
        case 'r':
            isRTR = 1;
            break;
        case 'R':
            isExended = 1;
            isRTR = 1;
            break;
        default:
            writeDebugStream(F("Unexpected type of transmit command\n"));
            writeDebugStream((const uint8_t*)buffer, length);
            writeDebugStream('\n');
            return ERROR_INVALID_COMMAND;

    }

    int offset = 1;

    canid_t id = 0;
    int idChars = isExended ? 8 : 3;
    for (int i=0; i<idChars; i++) {
        id <<= 4;
        id += hexCharToByte(buffer[offset++]);
    }
    if (isRTR) {
        id |= CAN_RTR_FLAG;
    }
    if (isExended) {
        id |= CAN_EFF_FLAG;
    }
    frame->can_id = id;

    __u8 dlc = hexCharToByte(buffer[offset++]);
    if (dlc > 8) {
        writeDebugStream(F("DLC > 8\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }
    if (dlc == 0) {
        writeDebugStream(F("DLC = 0\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }
    frame->can_dlc = dlc;

    if (!isRTR) {
        for (int i=0; i<dlc; i++) {
            char hiHex = buffer[offset++];
            char loHex = buffer[offset++];
            frame->data[i] = hexCharToByte(loHex) + (hexCharToByte(hiHex) << 4);
        }
    }

    return ERROR_OK;
}

CanHacker::ERROR CanHacker::createTransmit(const can_message *frame, char *buffer, const int length) {
    int offset;
    int len = frame->can_dlc;

    int isRTR = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;

    if (frame->can_id & CAN_ERR_FLAG) {
        return ERROR_ERROR_FRAME_NOT_SUPPORTED;
    } else if (frame->can_id & CAN_EFF_FLAG) {
        buffer[0] = isRTR ? 'R' : 'T';
        put_eff_id(buffer+1, frame->can_id & CAN_EFF_MASK);
        offset = 9;
    } else {
        buffer[0] = isRTR ? 'r' : 't';
        put_sff_id(buffer+1, frame->can_id & CAN_SFF_MASK);
        offset = 4;
    }

    buffer[offset++] = hex_asc_upper_lo(frame->can_dlc);

    if (!isRTR) {
        int i;
        for (i = 0; i < len; i++) {
            put_hex_byte(buffer + offset, frame->data[i]);
            offset += 2;
        }
    }

    if (_timestampEnabled) {
        uint16_t ts = getTimestamp();
        put_hex_byte(buffer + offset, ts >> 8);
        offset += 2;
        put_hex_byte(buffer + offset, ts);
        offset += 2;
    }

    buffer[offset++] = CR;
    buffer[offset] = '\0';

    if (offset >= length) {
        return ERROR_BUFFER_OVERFLOW;
    }

    return ERROR_OK;
}

bool CanHacker::readFrame(can_message &frame){
  if(can_instance->read(frame)){
    return true;
  }
  return false;
}

CanHacker::ERROR CanHacker::sendFrame(const can_message &frame) {
    return writeCan(frame);
}

CanHacker::ERROR CanHacker::receiveTransmitCommand(const char *buffer, const int length) {
    if (!isConnected()) {
        writeDebugStream(F("Transmit command while not connected\n"));
        return ERROR_NOT_CONNECTED;
    }

    if (_listenOnly) {
        return ERROR_LISTEN_ONLY;
    }

    can_message frame;
    ERROR error = parseTransmit(buffer, length, &frame);
    if (error != ERROR_OK) {
        return error;
    }
    bool sent = can_instance->write(frame);
    if (!sent) {
        return ERROR_SEND;
    }

    return writeStream(CR);
}

CanHacker::ERROR CanHacker::receiveTimestampCommand(const char *buffer, const int length) {
    if (length != 2) {
        writeStream(BEL);
        writeDebugStream(F("Timestamp command must be 2 bytes long\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }
    switch (buffer[1]) {
        case '0':
            _timestampEnabled = false;
            return writeStream(CR);
        case '1':
            _timestampEnabled = true;
            return writeStream(CR);
        default:
            writeStream(BEL);
            writeDebugStream(F("Timestamp cammand must have value 0 or 1\n"));
            writeDebugStream((const uint8_t*)buffer, length);
            writeDebugStream('\n');
            return ERROR_INVALID_COMMAND;
    }

    return ERROR_OK;
}

CanHacker::ERROR CanHacker::receiveCloseCommand(const char *buffer, const int length) {
    writeDebugStream(F("receiveCloseCommand\n"));

    if (length < 1) {
        return ERROR_INVALID_COMMAND;
    }

    if (buffer[0] != COMMAND_CLOSE_CAN_CHAN) {
        return ERROR_INVALID_COMMAND;
    }

    if (!isConnected()) {
        return writeStream(BEL);
    }
    ERROR error = disconnectCan();
    if (error != ERROR_OK) {
        return error;
    }
    return writeStream(CR);
}

CanHacker::ERROR CanHacker::receiveOpenCommand(const char *buffer, const int length) {

    if (length != 1) {
        return ERROR_INVALID_COMMAND;
    }

    if (buffer[0] != COMMAND_OPEN_CAN_CHAN) {
        return ERROR_INVALID_COMMAND;
    }

    writeDebugStream(F("receiveOpenCommand\n"));
    ERROR error = connectCan();
    if (error != ERROR_OK) {
        return error;
    }
    return writeStream(CR);
}

CanHacker::ERROR CanHacker::receiveListenOnlyCommand(const char *buffer, const int length) {

    if (length != 1) {
        return ERROR_INVALID_COMMAND;
    }

    if (buffer[0] != COMMAND_LISTEN_ONLY) {
        return ERROR_INVALID_COMMAND;
    }

    writeDebugStream(F("receiveListenOnlyCommand\n"));
    if (isConnected()) {
        writeStream(BEL);
        writeDebugStream(F("ListenOnly command cannot be called while connected\n"));
        return ERROR_CONNECTED;
    }
    _listenOnly = true;
    can_instance->setListenOnly(true);
    return writeStream(CR);
}

CanHacker::ERROR CanHacker::receiveSetAcrCommand(const char *buffer, const int length) {
    if (length != 9) {
        writeStream(BEL);
        writeDebugStream(F("ACR command must by 9 bytes long\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }
    uint32_t id = 0;
    for (int i=1; i<=8; i++) {
        id <<= 4;
        id += hexCharToByte(buffer[i]);
    }

    bool beenConnected = isConnected();
    ERROR error;

    if (beenConnected) {
        error = disconnectCan();
        if (error != ERROR_OK) {
            return error;
        }
    }

    if(!can_instance->setFilter(id)){
        writeDebugStream(F("Filter cannot be set.\n"));
        return ERROR_NOT_CONNECTED;
    }

    if (beenConnected) {
        error = (ERROR)isConnected();
        if (error != ERROR_OK) {
            return error;
        }
    }

    return writeStream(CR);
}

CanHacker::ERROR CanHacker::receiveSetAmrCommand(const char *buffer, const int length) {
    if (length != 9) {
        writeStream(BEL);
        writeDebugStream(F("AMR command must by 9 bytes long\n"));
        writeDebugStream((const uint8_t*)buffer, length);
        writeDebugStream('\n');
        return ERROR_INVALID_COMMAND;
    }
    uint32_t id = 0;
    for (int i=1; i<=8; i++) {
        id <<= 4;
        id += hexCharToByte(buffer[i]);
    }

    bool beenConnected = isConnected();
    ERROR error;

    if (beenConnected) {
        error = disconnectCan();
        if (error != ERROR_OK) {
            return error;
        }
    }

    error = (ERROR)can_instance->setFilterMask(id);
    if (error != ERROR_OK) {
        return error;
    }

    if (beenConnected) {
        error = connectCan();
        if (error != ERROR_OK) {
            return error;
        }
    }

    return writeStream(CR);
}

CanHacker::ERROR CanHacker::enableLoopback() {
    if (isConnected()) {
        writeDebugStream(F("Loopback cannot be changed while connected\n"));
        return ERROR_CONNECTED;
    }

    can_instance->enableLoopBack(true);

    return ERROR_OK;
}

CanHacker::ERROR CanHacker::disableLoopback() {
    if (isConnected()) {
        writeDebugStream(F("Loopback cannot be changed while connected\n"));
        return ERROR_CONNECTED;
    }

    can_instance->enableLoopBack(false);

    return ERROR_OK;
}
