#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ade7953_base/ade7953_base.h"

#include <vector>

namespace esphome {
namespace ade7953_i2c {

class AdE7953I2c : public ade7953_base::ADE7953, public i2c::I2CDevice {
 public:
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

};

}  // namespace ade7953_i2c
}  // namespace esphome
