#include "ade7953_spi.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ade7953_spi {

static const char *const TAG = "ade7953";

void AdE7953Spi::setup() {
  this->spi_setup();
  ade7953_base::ADE7953::setup();
}

void AdE7953Spi::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7953_spi:");
  LOG_PIN("  CS Pin: ", this->cs_);
  ade7953_base::ADE7953::dump_config();
}

// bool AdE7953Spi::ade_read_8(uint16_t reg, uint8_t *value) {
//   this->enable();
//   this->write_byte16(reg);
//   this->transfer_byte(0x80);
//   *value = this->read_byte();
//   this->disable();
//   return false;
// }

uint8_t AdE7953Spi::read_u8_register16_(uint16_t a_register) {
  uint8_t in;
  // this->read_register16(a_register, &in, sizeof(in));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0x80);
  in = this->read_byte();
  this->disable();
  return in;
}

// bool AdE7953Spi::ade_read_16(uint16_t reg, uint16_t *value) {
//   this->enable();
//   this->write_byte16(reg);
//   this->transfer_byte(0x80);
//   uint8_t recv[2];
//   this->read_array(recv, 2);
//   *value = encode_uint16(recv[0], recv[1]);
//   this->disable();
//   return false;
// }

uint16_t AdE7953Spi::read_u16_register16_(uint16_t a_register) {
  uint16_t in;
  // this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0x80);
  uint8_t recv[2];
  this->read_array(recv, 2);
  in = encode_uint16(recv[0], recv[1]);
  this->disable();
  return in;
}

int16_t AdE7953Spi::read_s16_register16_(uint16_t a_register){
  int16_t in;
  // this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0x80);
  uint8_t recv[2];
  this->read_array(recv, 2);
  in = encode_int16(recv[0], recv[1]);
  this->disable();
  return in;
}

// bool AdE7953Spi::ade_read_32(uint16_t reg, uint32_t *value) {
//   this->enable();
//   this->write_byte16(reg);
//   this->transfer_byte(0x80);
//   uint8_t recv[4];
//   this->read_array(recv, 4);
//   *value = encode_uint32(recv[0], recv[1], recv[2], recv[3]);
//   this->disable();
//   return false;
// }

uint32_t AdE7953Spi::read_u32_register16_(uint16_t a_register) {
  uint32_t in;
  // this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0x80);
  uint8_t recv[4];
  this->read_array(recv, 4);
  in = encode_uint32(recv[0], recv[1], recv[2], recv[3]);
  this->disable();
  return in;
}

int32_t AdE7953Spi::read_s32_register16_(uint16_t a_register) {
  int32_t in;
  // this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0x80);
  uint8_t recv[4];
  this->read_array(recv, 4);
  in = encode_int32(recv[0], recv[1], recv[2], recv[3]);
  this->disable();
  return in;
}

// bool AdE7953Spi::ade_write_8(uint16_t reg, uint8_t value) {
//   this->enable();
//   this->write_byte16(reg);
//   this->transfer_byte(0);
//   this->transfer_byte(value);
//   this->disable();
//   return false;
// }

void AdE7953Spi::write_u8_register16_(uint16_t a_register, uint8_t value){
  // this->write_register16(a_register, &value, sizeof(value));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0);
  this->transfer_byte(value);
  this->disable();
}

// bool AdE7953Spi::ade_write_16(uint16_t reg, uint16_t value) {
//   this->enable();
//   this->write_byte16(reg);
//   this->transfer_byte(0);
//   this->write_byte16(value);
//   this->disable();
//   return false;
// }

void AdE7953Spi::write_u16_register16_(uint16_t a_register, uint16_t value){
  // uint16_t out = convert_big_endian(value);
  // this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0);
  this->write_byte16(value);
  this->disable();

}

// bool AdE7953Spi::ade_write_32(uint16_t reg, uint32_t value) {
//   this->enable();
//   this->write_byte16(reg);
//   this->transfer_byte(0);
//   this->write_byte16(value >> 16);
//   this->write_byte16(value & 0xFFFF);
//   this->disable();
//   return false;
// }

void AdE7953Spi::write_u32_register16_(uint16_t a_register, uint32_t value){
  // uint32_t out = convert_big_endian(value);
  // this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
  this->enable();
  this->write_byte16(a_register);
  this->transfer_byte(0);
  this->write_byte16(value >> 16);
  this->write_byte16(value & 0xFFFF);
  this->disable();
}

// void AdE7953Spi::write_s32_register16_(uint16_t a_register, int32_t value) {
//   int32_t out = convert_big_endian(value);
//   this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
// }


}  // namespace ade7953_spi
}  // namespace esphome
