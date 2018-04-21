/**
 * Copyright (C) 2017 Mikkel Kroman <mk@maero.dk>
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef __SHT3X_I2C_H
#define __SHT3X_I2C_H

// Require the i2c library
#include <Wire.h>

#include <Arduino.h>

#define SHT3X_OK 0
#define SHT3X_ERROR_I2C 1
#define SHT3X_ERROR_TIMEOUT 2
#define SHT3X_ERROR_TEMP_CRC 3
#define SHT3X_ERROR_HUMIDITY_CRC 4

/// The SHT3X class wraps the I2C communication driver for a Sensiron SHT3x sensor.
class SHT3X {
public:
  /// Creates a new wrapper for an SHT3x I2C sensor on `address`.
  ///
  /// @param[in] address The I2C address of the SHT3x sensor.
  SHT3X(uint8_t address);

  /// Sends a soft reset command to the sensor.
  ///
  /// There's a 0.5ms - 1ms (min - max) soft reset delay on the sensor.
  /// This function will keep the CPU busy for 1ms before returning.
  int reset(void);

  /// Acquires a single-shot measurement from the sensor.
  ///
  /// @param[out] temperature The temperature in centigrade scale.
  /// @param[out] humidity    The humidity in relative humidity.
  ///
  /// @see GetErrorMessage for a user-readable error message.
  ///
  /// @return non-zero on failure, zero otherwise.
  int getMeasurement(double* temperature, double* humidity);

  /// Returns a user-readable error message for the given code.
  ///
  /// @param[in] error_code The error code.
  ///
  /// @return user-readable error message.
  static const char* GetErrorMessage(unsigned int error_code);

private:
  uint8_t _address;
};

#endif
