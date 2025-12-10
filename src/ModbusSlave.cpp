#include "ModbusSlave.h"

ModbusSlave::ModbusSlave(Stream& serial, uint8_t slaveId) 
  : _serial(serial), _slaveId(slaveId), _registers(nullptr), _regsCount(0) {}

void ModbusSlave::setRegisters(uint16_t* regs, uint16_t count) {
  _registers = regs;
  _regsCount = count;
}

void ModbusSlave::poll() {
  while (_serial.available()) {
    uint8_t b = _serial.read();
    if (_len < BUFFER_SIZE) {
      _buffer[_len++] = b;
      _lastByteTime = millis();
    }
  }

  if (_len > 0 && (millis() - _lastByteTime > 10)) {
    processFrame();
    _len = 0; // Сброс буфера 
  }
}

void ModbusSlave::processFrame() {
  if (_len < 4) return; 

  if (_buffer[0] != _slaveId) return;

  // CRC
  uint16_t receivedCRC = (_buffer[_len - 1] << 8) | _buffer[_len - 2];
  uint16_t calculatedCRC = calculateCRC(_buffer, _len - 2);

  uint16_t validCRC = ((uint16_t)_buffer[_len - 1] << 8) | _buffer[_len - 2]; 
  
  if (calculatedCRC != validCRC) return;

  uint8_t funcCode = _buffer[1];

  switch (funcCode) {
    case 0x03: // Read Holding Registers
      handleReadHoldingRegisters();
      break;
    case 0x06: // Write Single Register
      handleWriteSingleRegister();
      break;
    default:
      sendException(0x01); // Illegal Function
      break;
  }
}

void ModbusSlave::handleReadHoldingRegisters() {
  //[ID] [03] [AddrHi] [AddrLo] [QtyHi] [QtyLo] [CRC] [CRC]
  uint16_t startAddr = (_buffer[2] << 8) | _buffer[3];
  uint16_t quantity = (_buffer[4] << 8) | _buffer[5];

  if (startAddr + quantity > _regsCount) {
    sendException(0x02); // Illegal Data Address
    return;
  }

  //[ID] [03] [BytesCount] [DataHi] [DataLo]... [CRC] [CRC]
  uint8_t response[64];
  response[0] = _slaveId;
  response[1] = 0x03;
  response[2] = quantity * 2; // Количество байт данных

  int idx = 3;
  for (int i = 0; i < quantity; i++) {
    response[idx++] = highByte(_registers[startAddr + i]);
    response[idx++] = lowByte(_registers[startAddr + i]);
  }

  uint16_t crc = calculateCRC(response, idx);
  response[idx++] = lowByte(crc);
  response[idx++] = highByte(crc);

  _serial.write(response, idx);
}

void ModbusSlave::handleWriteSingleRegister() {
  //[ID] [06] [AddrHi] [AddrLo] [ValHi] [ValLo] [CRC] [CRC]
  uint16_t addr = (_buffer[2] << 8) | _buffer[3];
  uint16_t value = (_buffer[4] << 8) | _buffer[5];

  if (addr >= _regsCount) {
    sendException(0x02);
    return;
  }

  _registers[addr] = value;

  _serial.write(_buffer, _len - 2); 
  
  // Добавляем CRC 
  uint16_t crc = calculateCRC(_buffer, _len - 2);
  _serial.write(lowByte(crc));
  _serial.write(highByte(crc));
}

void ModbusSlave::sendException(uint8_t code) {
  uint8_t response[5];
  response[0] = _slaveId;
  response[1] = _buffer[1] | 0x80; // Error flag
  response[2] = code;
  
  uint16_t crc = calculateCRC(response, 3);
  response[3] = lowByte(crc);
  response[4] = highByte(crc);
  
  _serial.write(response, 5);
}

// Стандартный алгоритм CRC-16 для Modbus
uint16_t ModbusSlave::calculateCRC(uint8_t *buffer, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}