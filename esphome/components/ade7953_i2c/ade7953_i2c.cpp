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

uint8_t AdE7953I2c::read_u8_register16_(uint16_t a_register) {
  uint8_t in;
  this->read_register16(a_register, &in, sizeof(in));
  return in;
}

uint16_t AdE7953I2c::read_u16_register16_(uint16_t a_register) {
  uint16_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

int16_t AdE7953I2c::read_s16_register16_(uint16_t a_register){
  int16_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

uint32_t AdE7953I2c::read_u32_register16_(uint16_t a_register) {
  uint32_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

int32_t AdE7953I2c::read_s32_register16_(uint16_t a_register) {
  int32_t in;
  this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  return convert_big_endian(in);
}

void AdE7953I2c::write_u8_register16_(uint16_t a_register, uint8_t value){
  this->write_register16(a_register, &value, sizeof(value));
}

void AdE7953I2c::write_u16_register16_(uint16_t a_register, uint16_t value){
  uint16_t out = convert_big_endian(value);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

// void AdE7953I2c::write_s32_register16_(uint16_t a_register, int32_t value) {
//   int32_t out = convert_big_endian(value);
//   this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
// }

void AdE7953I2c::write_u32_register16_(uint16_t a_register, uint32_t value){
  uint32_t out = convert_big_endian(value);
  this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
}

}  // namespace ade7953_i2c
}  // namespace esphome
