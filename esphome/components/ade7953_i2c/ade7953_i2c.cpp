#include "ade7953_i2c.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ade7953_i2c {

static const char *const TAG = "ade7953";

void AdE7953I2c::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7953_i2c:");
  LOG_I2C_DEVICE(this);
  ade7953_base::ADE7953::dump_config();
}

void AdE7953I2c::read_u8_register16_(uint16_t reg, uint8_t *value) {
  this->read_register16(reg, value, 1);
}

void AdE7953I2c::read_u16_register16_(uint16_t reg, uint16_t *value) {
  uint16_t in;
  this->read_register16(reg, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  *value = convert_big_endian(in);
}

void AdE7953I2c::read_s16_register16_(uint16_t reg, int16_t *value) {
  int16_t in;
  this->read_register16(reg, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  *value = convert_big_endian(in);
}

void AdE7953I2c::read_u32_register16_(uint16_t reg, uint32_t *value) {
  uint32_t in;
  this->read_register16(reg, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  *value = convert_big_endian(in);
}

void AdE7953I2c::read_s32_register16_(uint16_t reg, int32_t *value) {
  int32_t in;
  this->read_register16(reg, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  *value = convert_big_endian(in);
}

void AdE7953I2c::write_u8_register16_(uint16_t reg, uint8_t value) {
  this->write_register16(reg, &value, sizeof(value));
}

void AdE7953I2c::write_u16_register16_(uint16_t reg, uint16_t value) {
  uint16_t out = convert_big_endian(value);
  this->write_register16(reg, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

void AdE7953I2c::write_u32_register16_(uint16_t reg, uint32_t value) {
  uint32_t out = convert_big_endian(value);
  this->write_register16(reg, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

// void AdE7953I2c::write_s32_register16_(uint16_t reg, int32_t value) {
//   int32_t out = convert_big_endian(value);
//   this->write_register16(reg, reinterpret_cast<uint8_t *>(&out), sizeof(out));
// }

}  // namespace ade7953_i2c
}  // namespace esphome
