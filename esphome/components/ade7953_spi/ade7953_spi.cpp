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

// The ADE7953 requires the following specifics in the SPI interface (see: datasheet)
// - in ade7953_spi.h
//   - spi::BIT_ORDER_MSB_FIRST
//   - spi::CLOCK_POLARITY_HIGH (CPOL = 1), in ade7953_spi.h  ("The MISO pin is an output from the ADE7953; data is shifted out on the falling edge of SCLK and should ...)
//   - spi::CLOCK_PHASE_TRAILING (CPHA = 1), in ade7953_spi.h (... be sampled by the external microcontroller on the rising edge." => RISING = TRAILING when CPOL = 1 )
//   - spi::DATA_RATE_1MHZ, max speed: 5MHz, 5MHz tested and works on Shelly Pro4PM
// - the registry id is always 16 bits (uint16_t), no conversion needed, sent first on MOSI
// - a full extra R/W-byte (8bits) needs to be sent (on MOSI) after the registry id, READ = 0x80 / WRITE = 0x0
// - no dummy bits needed between CMD+ADDR-phase and DATA-phase
// - 'Normal SPI' = 1 data lane
// - when using 'framework: esp-idf':
//   - a WRITE-transaction (on MOSI) can be completed in a single function call to 'write_cmd_addr_data()' by calling with:
//       - COMMAND-phase = registry (length: 16)
//       - ADDRESS-phase = R/W-byte (length: 8)
//       - DUMMY-phase = 0 (hardcoded in function, spi_esp_idf.cpp)
//       - WRITE-phase = data buffer (*uint8_t[]) + length
//       - READ-phase = 0 (hardcoded in function, spi_esp_idf.cpp)
//   - a READ-transaction (on MISO & MOSI) can be completed in a single function call to 'read_cmd_addr_data()' by calling with:
//       - COMMAND-phase = registry (length: 16)
//       - ADDRESS-phase = R/W-byte (length: 8)
//       - DUMMY-phase = 0 (hardcoded in function, spi_esp_idf.cpp)
//       - WRITE-phase = 0 (hardcoded in function, spi_esp_idf.cpp)
//       - READ-phase = data buffer (*uint8_t[]) + length
//   - alternatively,
//       - a WRITE-transaction: 3 consecutive calls to write_cmd_addr_data(data-phase-only).
//       - a READ-transaction: 2 calls to write_cmd_addr_data(data-phase-only) and 1 call to read_cmd_addr_data(data-phase-only).


void AdE7953Spi::write_u8_register16_(uint16_t reg, uint8_t value) {
#ifdef USE_ARDUINO
  this->enable();
  this->write_byte16(reg);
  this->transfer_byte(0);
  this->transfer_byte(value);
  this->disable();
#endif // USE_ARDUINO
#ifdef USE_ESP_IDF
  uint16_t cmd = reg;
  uint64_t rw_byte = 0x0;
  uint8_t dlen = 1;
  uint8_t buf[dlen] = {0};
  buf[0] = value;

  this->enable();
  this->write_cmd_addr_data(16, cmd, 8, rw_byte, (uint8_t *) &buf, dlen, 1);
  this->disable();
#endif  // USE_ESP_IDF
}

void AdE7953Spi::write_u16_register16_(uint16_t reg, uint16_t value) {
#ifdef USE_ARDUINO
  this->enable();
  this->write_byte16(reg);
  this->transfer_byte(0);
  this->write_byte16(value);
  this->disable();
#endif // USE_ARDUINO
#ifdef USE_ESP_IDF
  uint16_t cmd = reg;
  uint64_t rw_byte = 0x0;
  uint8_t dlen = 2;
  uint8_t buf[dlen] = {0};
  buf[0] = (value >> 8);
  buf[1] = value & 0xff;

  this->enable();
  this->write_cmd_addr_data(16, cmd, 8, rw_byte, (uint8_t *) &buf, dlen, 1);
  this->disable();
#endif  // USE_ESP_IDF
}

void AdE7953Spi::write_u32_register16_(uint16_t reg, uint32_t value) {
#ifdef USE_ARDUINO
  this->enable();
  this->write_byte16(reg);
  this->transfer_byte(0);
  this->write_byte16(value >> 16);
  this->write_byte16(value & 0xFFFF);
  this->disable();
#endif // USE_ARDUINO
#ifdef USE_ESP_IDF
  // Example of writing reg(2byte) + rw(1byte) + value as three
  // consecutive calls with only data-phase.
  // uint8_t buf[4] = {0};
  // this->enable();
  // buf[0] = (reg >> 8);
  // buf[1] = reg & 0xff;
  // this->write_cmd_addr_data(0, 0, 0, 0, (uint8_t *) &buf, 2, 1);
  // buf[0] = 0x0;
  // this->write_cmd_addr_data(0, 0, 0, 0, (uint8_t *) &buf, 1, 1);
  // buf[0] = (value >> 24);
  // buf[1] = (value >> 16);
  // buf[2] = (value >> 8);
  // buf[3] = value & 0xff;
  // this->write_cmd_addr_data(0, 0, 0, 0, (uint8_t *) &buf, 4, 1);
  // this->disable();

  uint16_t cmd = reg;
  uint64_t rw_byte = 0x0;
  uint8_t dlen = 4;
  uint8_t buf[dlen] = {0};
  buf[0] = (value >> 24);
  buf[1] = (value >> 16);
  buf[2] = (value >> 8);
  buf[3] = value & 0xff;

  this->enable();
  this->write_cmd_addr_data(16, cmd, 8, rw_byte, (uint8_t *) &buf, dlen, 1);
  this->disable();
#endif  // USE_ESP_IDF
}

// void AdE7953Spi::write_s32_register16_(uint16_t a_register, int32_t value) {
//   int32_t out = convert_big_endian(value);
//   this->write_register16(a_register, reinterpret_cast<uint8_t *>(&out), sizeof(out));
// }

void AdE7953Spi::read_u8_register16_(uint16_t reg, uint8_t *value) {
#ifdef USE_ARDUINO
  this->enable();
  this->write_byte16(reg);
  this->transfer_byte(0x80);
  *value = this->read_byte();
  this->disable();
#endif // USE_ARDUINO
#ifdef USE_ESP_IDF
  uint16_t cmd = reg;
  uint64_t rw = 0x80;
  uint8_t dlen = 1;
  uint8_t buf[dlen] = {0};

  this->enable();
  this->read_cmd_addr_data(16, cmd, 8, rw, (uint8_t *) &buf, dlen, 1);
  this->disable();

  *value = buf[0];
#endif  // USE_ESP_IDF
}

void AdE7953Spi::read_u16_register16_(uint16_t reg, uint16_t *value) {
#ifdef USE_ARDUINO
  this->enable();
  this->write_byte16(reg);
  this->transfer_byte(0x80);
  uint8_t recv[2];
  this->read_array(recv, 2);
  *value = encode_uint16(recv[0], recv[1]);
  this->disable();
#endif // USE_ARDUINO
#ifdef USE_ESP_IDF
  uint16_t cmd = reg;
  uint64_t rw = 0x80;
  uint8_t dlen = 2;
  uint8_t buf[dlen] = {0};

  this->enable();
  this->read_cmd_addr_data(16, cmd, 8, rw, (uint8_t *) &buf, dlen, 1);
  this->disable();

  *value = (static_cast<uint16_t>(buf[0]) << 8) | (static_cast<uint16_t>(buf[1]));
#endif  // USE_ESP_IDF
}

void AdE7953Spi::read_s16_register16_(uint16_t reg, int16_t *value){
  // int16_t in;
  // // this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  // this->enable();
  // this->write_byte16(a_register);
  // this->transfer_byte(0x80);
  // uint8_t recv[2];
  // this->read_array(recv, 2);
  // // in = encode_int16(recv[0], recv[1]);
  // in = 0;
  // this->disable();
  // return in;
}

void AdE7953Spi::read_u32_register16_(uint16_t reg, uint32_t *value) {
#ifdef USE_ARDUINO
  this->enable();
  this->write_byte16(reg);
  this->transfer_byte(0x80);
  uint8_t recv[4];
  this->read_array(recv, 4);
  *value = encode_uint32(recv[0], recv[1], recv[2], recv[3]);
  this->disable();
#endif // USE_ARDUINO
#ifdef USE_ESP_IDF
  // Example of writing reg(2byte) + rw(1byte) and reading value as three
  // consecutive calls with only data-phase.
  // uint8_t buf[4] = {0};
  // this->enable();
  // buf[0] = (reg >> 8);
  // buf[1] = reg & 0xff;
  // this->write_cmd_addr_data(0, 0, 0, 0, (uint8_t *) &buf, 2, 1);
  // buf[0] = 0x80;
  // this->write_cmd_addr_data(0, 0, 0, 0, (uint8_t *) &buf, 1, 1);
  // this->read_cmd_addr_data(0, 0, 0, 0, (uint8_t *) &buf, 4, 1);
  // this->disable();
  // *value = (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) | (static_cast<uint32_t>(buf[2]) << 8) | (static_cast<uint32_t>(buf[3]));

  uint16_t cmd = reg;
  uint64_t rw = 0x80;
  uint8_t dlen = 4;
  uint8_t buf[dlen] = {0};

  this->enable();
  this->read_cmd_addr_data(16, cmd, 8, rw, (uint8_t *) &buf, dlen, 1);
  this->disable();

  *value = (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) | (static_cast<uint32_t>(buf[2]) << 8) | (static_cast<uint32_t>(buf[3]));
#endif  // USE_ESP_IDF
}

void AdE7953Spi::read_s32_register16_(uint16_t reg, int32_t *value) {
  // int32_t in;
  // // this->read_register16(a_register, reinterpret_cast<uint8_t *>(&in), sizeof(in));
  // this->enable();
  // this->write_byte16(a_register);
  // this->transfer_byte(0x80);
  // uint8_t recv[4];
  // this->read_array(recv, 4);
  // // in = encode_int32(recv[0], recv[1], recv[2], recv[3]);
  // in = 0;
  // this->disable();
  // return in;
}



}  // namespace ade7953_spi
}  // namespace esphome
