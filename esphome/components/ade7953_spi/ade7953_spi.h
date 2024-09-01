#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ade7953_base/ade7953_base.h"

#include <vector>

namespace esphome {
namespace ade7953_spi {

class AdE7953Spi : public ade7953_base::ADE7953,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;

  void dump_config() override;

 protected:
  void read_u8_register16_(uint16_t reg, uint8_t *value) override;
  void read_s16_register16_(uint16_t reg, int16_t *value) override;
  void read_u16_register16_(uint16_t reg, uint16_t *value) override;
  void read_s32_register16_(uint16_t reg, int32_t *value) override;
  void read_u32_register16_(uint16_t reg, uint32_t *value) override;

  void write_u8_register16_(uint16_t reg, uint8_t value) override;
  void write_u16_register16_(uint16_t reg, uint16_t value) override;
  void write_u32_register16_(uint16_t reg, uint32_t value) override;
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
