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
    // this->ade_write_8(0x0010, 0x04);
    this->ade_write_8(0x00FE, 0xAD);
    this->ade_write_16(0x0120, 0x0030);
    // Set gains
    this->ade_write_8(PGA_V_8, pga_v_);
    this->ade_write_8(PGA_IA_8, pga_ia_);
    this->ade_write_8(PGA_IB_8, pga_ib_);
    this->ade_write_32(AVGAIN_32, vgain_);
    this->ade_write_32(AIGAIN_32, aigain_);
    this->ade_write_32(BIGAIN_32, bigain_);
    this->ade_write_32(AWGAIN_32, awgain_);
    this->ade_write_32(BWGAIN_32, bwgain_);
    // Read back gains for debugging
    this->ade_read_8(PGA_V_8, &pga_v_);
    this->ade_read_8(PGA_IA_8, &pga_ia_);
    this->ade_read_8(PGA_IB_8, &pga_ib_);
    this->ade_read_32(AVGAIN_32, &vgain_);
    this->ade_read_32(AIGAIN_32, &aigain_);
    this->ade_read_32(BIGAIN_32, &bigain_);
    this->ade_read_32(AWGAIN_32, &awgain_);
    this->ade_read_32(BWGAIN_32, &bwgain_);
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

#define ADE_PUBLISH_(name, val, factor) \
  if (err == 0 && this->name##_sensor_) { \
    float value = (val) / (factor); \
    this->name##_sensor_->publish_state(value); \
  }
#define ADE_PUBLISH(name, val, factor) ADE_PUBLISH_(name, val, factor)

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

void ADE7953::update() {
  if (!this->is_setup_)
    return;

  bool err;

  uint32_t interrupts_a = 0;
  uint32_t interrupts_b = 0;
  if (this->irq_pin_ != nullptr) {
    // Read and reset interrupts
    this->ade_read_32(0x032E, &interrupts_a);
    this->ade_read_32(0x0331, &interrupts_b);
  }

  uint32_t val;
  uint16_t val_16;
  uint16_t reg;

  // Power factor
  err = this->ade_read_16(0x010A, &val_16);
  ADE_PUBLISH(power_factor_a, (int16_t) val_16, (0x7FFF / 100.0f));
  err = this->ade_read_16(0x010B, &val_16);
  ADE_PUBLISH(power_factor_b, (int16_t) val_16, (0x7FFF / 100.0f));

  float pref = ADE7953_PREF;
  if (this->use_acc_energy_regs_) {
    const uint32_t now = millis();
    const auto diff = now - this->last_update_;
    this->last_update_ = now;
    // prevent DIV/0
    pref = ADE7953_WATTSEC_PREF * (diff < 10 ? 10 : diff) / 1000;
    ESP_LOGD(TAG, "ADE7953::update() diff=%" PRIu32 " pref=%f", diff, pref);
  }

  // Apparent power
  reg = this->use_acc_energy_regs_ ? 0x0322 : 0x0310;
  err = this->ade_read_32(reg, &val);
  ADE_PUBLISH(apparent_power_a, (int32_t) val, pref);
  err = this->ade_read_32(reg + 1, &val);
  ADE_PUBLISH(apparent_power_b, (int32_t) val, pref);

  // Active power
  this->update_sensor_from_s32_register16_(this->active_power_a_sensor_, 0x031E, [](float val) { return val / pref; });
  this->update_sensor_from_s32_register16_(this->active_power_b_sensor_, 0x031F, [](float val) { return val / pref; });
  // reg = this->use_acc_energy_regs_ ? 0x031E : 0x0312;
  // err = this->ade_read_32(reg, &val);
  // ADE_PUBLISH(active_power_a, (int32_t) val, pref);
  // err = this->ade_read_32(reg + 1, &val);
  // ADE_PUBLISH(active_power_b, (int32_t) val, pref);

  // Reactive power
  reg = this->use_acc_energy_regs_ ? 0x0320 : 0x0314;
  err = this->ade_read_32(reg, &val);
  ADE_PUBLISH(reactive_power_a, (int32_t) val, pref);
  err = this->ade_read_32(reg + 1, &val);
  ADE_PUBLISH(reactive_power_b, (int32_t) val, pref);

  // Current
  this->update_sensor_from_u32_register16_(this->current_a_sensor_, 0x031A, [](float val) { return val / ADE7953_IREF; });
  this->update_sensor_from_u32_register16_(this->current_b_sensor_, 0x031B, [](float val) { return val / ADE7953_IREF; });

  // Voltage
  this->update_sensor_from_u32_register16_(this->voltage_sensor_, 0x031C, [](float val) { return val / ADE7953_UREF; });
  
  // Frequency
  err = this->ade_read_16(0x010E, &val_16);
  ADE_PUBLISH(frequency, 223750.0f, 1 + val_16);

  // Forward active energy
  err = this->ade_read_32(0x31E, &val);
  ADE_PUBLISH(forward_active_energy_a, (int32_t) val, pref);
  err = this->ade_read_32(0x31F, &val);
  ADE_PUBLISH(forward_active_energy_b, (int32_t) val, pref);
}

}  // namespace ade7953_base
}  // namespace esphome
