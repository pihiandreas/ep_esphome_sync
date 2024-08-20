#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ade7953_base/ade7953_base.h"

#include <vector>

namespace esphome {
namespace ade7953_spi {

/// missing encode_intX - function (copy from core/helpers.h) 
/// Encode a 16-bit signed value given the most and least significant byte.
constexpr int16_t encode_int16(uint8_t msb, uint8_t lsb) {
  return (static_cast<int16_t>(msb) << 8) | (static_cast<int16_t>(lsb));
}

/// Encode a 32-bit signed value given four bytes in most to least significant byte order.
constexpr int32_t encode_int32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
  return (static_cast<int32_t>(byte1) << 24) | (static_cast<int32_t>(byte2) << 16) |
         (static_cast<int32_t>(byte3) << 8) | (static_cast<int32_t>(byte4));
}


class AdE7953Spi : public ade7953_base::ADE7953,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;

  void dump_config() override;

 protected:
  uint8_t read_u8_register16_(uint16_t a_register);
  int16_t read_s16_register16_(uint16_t a_register);
  uint16_t read_u16_register16_(uint16_t a_register);
  int32_t read_s32_register16_(uint16_t a_register);
  uint32_t read_u32_register16_(uint16_t a_register);

  void write_u8_register16_(uint16_t a_register, uint8_t value);
  void write_u16_register16_(uint16_t a_register, uint16_t value);
  void write_u32_register16_(uint16_t a_register, uint32_t value);
  // void write_s32_register16_(uint16_t a_register, int32_t value);
  
  // bool ade_write_8(uint16_t reg, uint8_t value) override;
  // bool ade_write_16(uint16_t reg, uint16_t value) override;
  // bool ade_write_32(uint16_t reg, uint32_t value) override;
  // bool ade_read_8(uint16_t reg, uint8_t *value) override;
  // bool ade_read_16(uint16_t reg, uint16_t *value) override;
  // bool ade_read_32(uint16_t reg, uint32_t *value) override;
};

}  // namespace ade7953_spi
}  // namespace esphome
