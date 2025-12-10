#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <Arduino.h>

class ModbusSlave {
  public:
    ModbusSlave(Stream& serial, uint8_t slaveId);

    void setRegisters(uint16_t* regs, uint16_t count);

    void poll();

  private:
    Stream& _serial;
    uint8_t _slaveId;
    uint16_t* _registers;
    uint16_t _regsCount;

    // буфер
    static const int BUFFER_SIZE = 64;
    uint8_t _buffer[BUFFER_SIZE];
    int _len = 0;
    unsigned long _lastByteTime = 0;

    uint16_t calculateCRC(uint8_t* buf, int length);
    void processFrame();
    void handleReadHoldingRegisters();
    void handleWriteSingleRegister();
    void sendException(uint8_t code);
};

#endif