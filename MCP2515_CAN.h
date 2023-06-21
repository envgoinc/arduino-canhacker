
#ifndef MCP2515_CAN_H_
#define MCP2515_CAN_H_

#include <can.h>
#include <mcp2515.h>


typedef enum RXQUEUE_TABLE {
  RX_SIZE_2 = (uint16_t)2,
  RX_SIZE_4 = (uint16_t)4,
  RX_SIZE_8 = (uint16_t)8,
  RX_SIZE_16 = (uint16_t)16,
  RX_SIZE_32 = (uint16_t)32,
  RX_SIZE_64 = (uint16_t)64,
  RX_SIZE_128 = (uint16_t)128,
  RX_SIZE_256 = (uint16_t)256,
  RX_SIZE_512 = (uint16_t)512,
  RX_SIZE_1024 = (uint16_t)1024
} RXQUEUE_TABLE;

typedef enum TXQUEUE_TABLE {
  TX_SIZE_2 = (uint16_t)2,
  TX_SIZE_4 = (uint16_t)4,
  TX_SIZE_8 = (uint16_t)8,
  TX_SIZE_16 = (uint16_t)16,
  TX_SIZE_32 = (uint16_t)32,
  TX_SIZE_64 = (uint16_t)64,
  TX_SIZE_128 = (uint16_t)128,
  TX_SIZE_256 = (uint16_t)256,
  TX_SIZE_512 = (uint16_t)512,
  TX_SIZE_1024 = (uint16_t)1024
} TXQUEUE_TABLE;

typedef enum CAN_ERROR{
  ERROR_OK,
  ERROR_CONNECTED,
  ERROR_NOT_CONNECTED,
  ERROR_UNKNOWN_COMMAND,
  ERROR_INVALID_COMMAND,
  ERROR_ERROR_FRAME_NOT_SUPPORTED,
  ERROR_BUFFER_OVERFLOW,
  ERROR_SERIAL_TX_OVERRUN,
  ERROR_LISTEN_ONLY,
  ERROR_MCP2515_INIT,
  ERROR_MCP2515_CONFIG,
  ERROR_MCP2515_BITRATE,
  ERROR_MCP2515_SET_MODE,
  ERROR_MCP2515_SEND,
  ERROR_MCP2515_READ,
  ERROR_MCP2515_FILTER,
  ERROR_MCP2515_ERRIF,
  ERROR_MCP2515_MERRF,
  ERROR_MCP2515_WAKIF
}CAN_ERROR;


class MCP2515_CAN{
  public:
    // enum CAN_CLOCK {
    //   MCP_20MHZ,
    //   MCP_16MHZ,
    //   MCP_8MHZ
    // };

    // typedef enum ERROR {
    //   ERROR_OK,
    //   ERROR_CONNECTED,
    //   ERROR_NOT_CONNECTED,
    //   ERROR_UNKNOWN_COMMAND,
    //   ERROR_INVALID_COMMAND,
    //   ERROR_ERROR_FRAME_NOT_SUPPORTED,
    //   ERROR_BUFFER_OVERFLOW,
    //   ERROR_SERIAL_TX_OVERRUN,
    //   ERROR_LISTEN_ONLY,
    //   ERROR_MCP2515_INIT,
    //   ERROR_MCP2515_CONFIG,
    //   ERROR_MCP2515_BITRATE,
    //   ERROR_MCP2515_SET_MODE,
    //   ERROR_MCP2515_SEND,
    //   ERROR_MCP2515_READ,
    //   ERROR_MCP2515_FILTER,
    //   ERROR_MCP2515_ERRIF,
    //   ERROR_MCP2515_MERRF,
    //   ERROR_MCP2515_WAKIF
    // };

    //typedef struct can_frame can_frame;
    


    // Default buffer sizes are set to 64 and 16. But this can be changed by using constructor in main code.
    MCP2515_CAN(uint8_t cs, const uint32_t spi_clock = 0, RXQUEUE_TABLE rxSize = RX_SIZE_64, TXQUEUE_TABLE txSize = TX_SIZE_16); // ADDED
    void begin(const int INT_PIN); // ADDED
    void setError(CAN_ERROR error); // ADDED
    CAN_ERROR getError(); // ADDED
    void setClock(CAN_CLOCK clock); // ADDED
    void setBaudRate(CAN_SPEED speed); // ADDED
    bool read(can_frame &CAN_rx_msg); // ADDED
    bool write(const can_frame &CAN_tx_msg, bool sendMB = false); //ADDED
    // Manually set filter and mask parameters
    bool setFilter(const uint32_t filter);//ADDED
    bool setFilterMask(const uint32_t mask);//ADDED
    void setListenOnly(bool listenOnly);//ADDED
    void enableLoopBack(bool yes = 1); //ADDED 
    void processInterrupt();
    CAN_ERROR connectCan(); //ADDED
    CAN_ERROR disconnectCan(); //ADDED 
    bool isConnected();
  
    // These are public because these are also used from interrupts.
    typedef struct RingbufferTypeDef {
      volatile uint16_t head;
      volatile uint16_t tail;
      uint16_t size;
      volatile can_frame *buffer;
    } RingbufferTypeDef;

    RingbufferTypeDef rxRing;
    //RingbufferTypeDef txRing;

    bool addToRingBuffer(RingbufferTypeDef &ring, const can_frame &msg);
    bool removeFromRingBuffer(RingbufferTypeDef &ring, can_frame &msg);

    volatile can_frame *rx_buffer;
    //volatile can_frame *tx_buffer;

  protected:
    uint16_t sizeRxBuffer;
    //uint16_t sizeTxBucheckErrorCounterffer;
  
  private:
    MCP2515 *_mcp2515;
    uint8_t _cs;
    CAN_STATE _state;
    CAN_ERROR _err;
    CAN_CLOCK _canClock = MCP_16MHZ;
    CAN_SPEED _bitrate;
    bool _timestampEnabled = false;
    bool _listenOnly = false;
    bool _loopback = false;
    bool _isConnected = false;
    bool _bufferOverrun = false;
    uint8_t _rxErrorCount = 0;
    static const char CR  = '\r';
    static const char BEL = 7;
    static const uint16_t TIMESTAMP_LIMIT = 0xEA60;

    bool isInitialized() { return rx_buffer != 0; } // ADDED
    void initRingBuffer(RingbufferTypeDef &ring, volatile can_frame *buffer, uint32_t size); // ADDED
    void initializeBuffers(void); // ADDED
    bool isRingBufferEmpty(RingbufferTypeDef &ring); // ADDED
    uint32_t ringBufferCount(RingbufferTypeDef &ring); // ADDED

    CAN_ERROR createBusState(const char state, char *buffer, const int length);
    CAN_ERROR createErrorStatus(const char error, char *buffer, const int length);
    CAN_ERROR checkErrorCounter();
    void pollReceiveCan();
    void processError();
    CAN_ERROR receiveCan(const MCP2515::RXBn rxBuffer);
};




#endif //MCP2515_CAN_H_