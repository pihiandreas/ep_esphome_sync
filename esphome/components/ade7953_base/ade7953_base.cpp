#include "ade7953_base.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace ade7953_base {

static const char *const TAG = "ade7953";

// static const float ADE_POWER_FACTOR = 154.0f;
// static const float ADE_WATTSEC_POWER_FACTOR = ADE_POWER_FACTOR * ADE_POWER_FACTOR / 3600;

// from https://github.com/arendst/Tasmota/blob/development/tasmota/tasmota_xnrg_energy/xnrg_07_ade7953.ino
// #define ADE7953_PREF              1540       // 4194304 / (1540 / 1000) = 2723574 (= WGAIN, VAGAIN and VARGAIN)
// #define ADE7953_UREF              26000      // 4194304 / (26000 / 10000) = 1613194 (= VGAIN)
// #define ADE7953_IREF              10000      // 4194304 / (10000 / 10000) = 4194303 (= IGAIN, needs to be different than 4194304 in order to use calib.dat)

static const float ADE7953_PREF = 154.0f;
static const float ADE7953_UREF = 26000.0f;
static const float ADE7953_IREF = 100000.0f;
static const float ADE7953_WATTSEC_PREF = ADE7953_PREF * ADE7953_PREF / 3600.0f;

void ADE7953::setup() {
  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
  }

  // The chip might take up to 100ms to initialise
  this->set_timeout(100, [this]() {
    // this->write_u8_register16_(0x0010, 0x04);
    this->write_u8_register16_(0x00FE, 0xAD);    // Unlock
    this->write_u16_register16_(0x0120, 0x0030); // see: ADE7953 Data Sheet Rev. C | Page 18 of 72 | ADE7953 POWER-UP PROCEDURE

    
    // Set gains
    this->write_u8_register16_(PGA_V_8, pga_v_);
    this->write_u8_register16_(PGA_IA_8, pga_ia_);
    this->write_u8_register16_(PGA_IB_8, pga_ib_);
    this->write_u32_register16_(AVGAIN_32, vgain_);
    this->write_u32_register16_(AIGAIN_32, aigain_);
    this->write_u32_register16_(BIGAIN_32, bigain_);
    this->write_u32_register16_(AWGAIN_32, awgain_);
    this->write_u32_register16_(BWGAIN_32, bwgain_);

    // IRMSOS 
    this->write_s32_register16_(0x0386, 0xF7D6); // AIRMSOS
    this->write_s32_register16_(0x0392, 0xF7D6); // BIRMSOS

    // Read back gains for debugging
    pga_v_ = this->read_u8_register16_(PGA_V_8);
    pga_ia_ = this->read_u8_register16_(PGA_IA_8);
    pga_ib_ = this->read_u8_register16_(PGA_IB_8);
    vgain_ = this->read_u32_register16_(AVGAIN_32);
    aigain_ = this->read_u32_register16_(AIGAIN_32);
    bigain_ = this->read_u32_register16_(BIGAIN_32);
    awgain_ = this->read_u32_register16_(AWGAIN_32);
    bwgain_ = this->read_u32_register16_(BWGAIN_32);
    this->last_update_ = millis();
    this->is_setup_ = true;
  });
}

void ADE7953::dump_config() {
  LOG_PIN("  IRQ Pin: ", irq_pin_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Voltage Sensor", this->voltage_sensor_);
  LOG_SENSOR("  ", "Current A Sensor", this->current_a_sensor_);
  LOG_SENSOR("  ", "Current B Sensor", this->current_b_sensor_);
  LOG_SENSOR("  ", "Power Factor A Sensor", this->power_factor_a_sensor_);
  LOG_SENSOR("  ", "Power Factor B Sensor", this->power_factor_b_sensor_);
  LOG_SENSOR("  ", "Apparent Power A Sensor", this->apparent_power_a_sensor_);
  LOG_SENSOR("  ", "Apparent Power B Sensor", this->apparent_power_b_sensor_);
  LOG_SENSOR("  ", "Active Power A Sensor", this->active_power_a_sensor_);
  LOG_SENSOR("  ", "Active Power B Sensor", this->active_power_b_sensor_);
  LOG_SENSOR("  ", "Reactive Power A Sensor", this->reactive_power_a_sensor_);
  LOG_SENSOR("  ", "Reactive Power B Sensor", this->reactive_power_b_sensor_);
  LOG_SENSOR("  ", "Forward Active Energy A Sensor", this->forward_active_energy_a_sensor_);
  LOG_SENSOR("  ", "Forward Active Energy B Sensor", this->forward_active_energy_b_sensor_);
  ESP_LOGCONFIG(TAG, "  USE_ACC_ENERGY_REGS: %d", this->use_acc_energy_regs_);
  ESP_LOGCONFIG(TAG, "  PGA_V_8: 0x%X", pga_v_);
  ESP_LOGCONFIG(TAG, "  PGA_IA_8: 0x%X", pga_ia_);
  ESP_LOGCONFIG(TAG, "  PGA_IB_8: 0x%X", pga_ib_);
  ESP_LOGCONFIG(TAG, "  VGAIN_32: 0x%08jX", (uintmax_t) vgain_);
  ESP_LOGCONFIG(TAG, "  AIGAIN_32: 0x%08jX", (uintmax_t) aigain_);
  ESP_LOGCONFIG(TAG, "  BIGAIN_32: 0x%08jX", (uintmax_t) bigain_);
  ESP_LOGCONFIG(TAG, "  AWGAIN_32: 0x%08jX", (uintmax_t) awgain_);
  ESP_LOGCONFIG(TAG, "  BWGAIN_32: 0x%08jX", (uintmax_t) bwgain_);
}

template<typename F>
void ADE7953::update_sensor_from_u32_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_u32_register16_(a_register);
  sensor->publish_state(f(val));
}

template<typename F>
void ADE7953::update_sensor_from_s32_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_s32_register16_(a_register);
  sensor->publish_state(f(val));
}

template<typename F>
void ADE7953::update_sensor_from_s16_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_s16_register16_(a_register);
  sensor->publish_state(f(val));
}

template<typename F>
void ADE7953::update_sensor_from_u16_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_u16_register16_(a_register);
  sensor->publish_state(f(val));
}

template<typename F>
void ADE7953::update_sensor_from_u8_register16_(sensor::Sensor *sensor, uint16_t a_register, F &&f) {
  if (sensor == nullptr) {
    return;
  }

  float val = this->read_u8_register16_(a_register);
  sensor->publish_state(f(val));
}

void ADE7953::update() {
  if (!this->is_setup_)
    return;

  if (this->irq_pin_ != nullptr) {
    // Read and reset interrupts
    this->read_u8_register16_(0x32E);
    this->read_u8_register16_(0x331);
  }

  // Power factor
  this->update_sensor_from_s16_register16_(this->power_factor_a_sensor_, 0x010A, [](float val) { return val / (0x7FFF / 100.0f); });
  this->update_sensor_from_s16_register16_(this->power_factor_b_sensor_, 0x010B, [](float val) { return val / (0x7FFF / 100.0f); });

  const uint32_t now = millis();
  const auto diff = now - this->last_update_;
  this->last_update_ = now;
  // prevent DIV/0
  float pref = ADE7953_WATTSEC_PREF * (diff < 10 ? 10 : diff) / 1000.0f;
  float eref = ADE7953_WATTSEC_PREF / 3600.0f;

  // Active power & Forward active energy (both from 0x031E / 0x031F)
  float aenergya = this->read_s32_register16_(0x031E);
  this->active_power_a_sensor_->publish_state(aenergya / pref);
  this->forward_active_energy_a_total += (aenergya / eref);
  this->forward_active_energy_a_sensor_->publish_state(this->forward_active_energy_a_total);

  float aenergyb = this->read_s32_register16_(0x031F);
  this->active_power_b_sensor_->publish_state(aenergyb / pref);
  this->forward_active_energy_b_total += (aenergyb / eref);
  this->forward_active_energy_b_sensor_->publish_state(this->forward_active_energy_b_total);

  // Reactive power
  this->update_sensor_from_s32_register16_(this->reactive_power_a_sensor_, 0x0320, [pref](float val) { return val / pref; });
  this->update_sensor_from_s32_register16_(this->reactive_power_b_sensor_, 0x0321, [pref](float val) { return val / pref; });

  // Apparent power
  this->update_sensor_from_s32_register16_(this->apparent_power_a_sensor_, 0x0322, [pref](float val) { return val / pref; });
  this->update_sensor_from_s32_register16_(this->apparent_power_b_sensor_, 0x0323, [pref](float val) { return val / pref; });

  // Current
  this->update_sensor_from_u32_register16_(this->current_a_sensor_, 0x031A, [](float val) { return val / ADE7953_IREF; });
  this->update_sensor_from_u32_register16_(this->current_b_sensor_, 0x031B, [](float val) { return val / ADE7953_IREF; });

  // Voltage
  this->update_sensor_from_u32_register16_(this->voltage_sensor_, 0x031C, [](float val) { return val / ADE7953_UREF; });
  
  // Frequency
  this->update_sensor_from_u16_register16_(this->frequency_sensor_, 0x010E, [](float val) { return 223750.0f / (1 + val); });

}

}  // namespace ade7953_base
}  // namespace esphome
