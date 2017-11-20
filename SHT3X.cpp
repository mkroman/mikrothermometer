#include "SHT3X.hpp"

#define SHT3X_RESPONSE_LENGTH 6
#define SHT3X_CRC8_POLYNOMIAL 0x31

SHT3X::SHT3X(uint8_t address) : _address(address) {
  Wire.begin();
}

// Run-time calculation of CRC8 polynomial
//
// TODO: Pre-computed polynomial table
// https://github.com/ControlEverythingCommunity/CE_ARDUINO_LIB/blob/master/SHT30/SHT30.cpp#L196
uint8_t crc8(const uint8_t* data, int length) {
  // Extracted from the datasheet:
  // Width: 8-bit
  // Polynomial: 0x31 (x⁸ + x⁵ + x⁴ + 1)
  // Initialization: 0xFF
  // Final XOR: 0x00
  const uint8_t POLYNOMIAL = 0x31;
  uint8_t crc = 0xFF;

  for (int j = length; j; --j) {
    crc ^= *data++;
    
    for (int i = 8; i; --i) {
      crc = (crc & 0x80)
        ? (crc << 1) ^ POLYNOMIAL
        : (crc << 1);
    }
  }
  
  return crc;
}

int SHT3X::reset() {
  Wire.beginTransmission(_address);
  Wire.write(0x30);
  Wire.write(0xA2);
  Wire.endTransmission();

  // A soft reset takes between 0.5 and 1ms.
  delay(1);
}

int SHT3X::getMeasurement(float* temperature, float* humidity) {
  uint8_t data[SHT3X_RESPONSE_LENGTH];
  byte result;

  Wire.beginTransmission(_address);

  // Send a single-shot measurement command
  Wire.write(0x2C); // Clock-stretching = Enabled
  Wire.write(0x06); // Repeatability = High

  if ((result = Wire.endTransmission()) != 0) {
    return SHT3X_ERROR_I2C;
  }

  // Delay for 15ms. According to the datasheet, a high repeatability measurement can take up to 15ms.
  delay(15);

  // Request the measurement packet (6 bytes.)
  result = Wire.requestFrom(_address, (size_t)SHT3X_RESPONSE_LENGTH);

  if (result != SHT3X_RESPONSE_LENGTH) {
    return SHT3X_ERROR_I2C;
  }

  for (int i = 0; i < SHT3X_RESPONSE_LENGTH; i++) {
    data[i] = Wire.read();
  }

  if (data[2] != crc8(data, 2)) {
    return SHT3X_ERROR_TEMP_CRC;
  } else {
    *temperature = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
  }

  if (data[5] != crc8(data + 3, 2)) {
    return SHT3X_ERROR_HUMIDITY_CRC;
  } else {
    *humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);
  }

  return SHT3X_OK;
}

const char* SHT3X::GetErrorMessage(unsigned int error_code) {
  switch (error_code) {
    case SHT3X_ERROR_I2C:
      return "There was a fatal I2C error";
      break;
    case SHT3X_ERROR_TIMEOUT:
      return "Communication to the sensor timed out";
      break;
    case SHT3X_ERROR_TEMP_CRC:
      return "CRC validation failed for temperature measurement";
      break;
    case SHT3X_ERROR_HUMIDITY_CRC:
      return "CRC validation failed for humidity measurement";
      break;
  }

  return NULL;
}

