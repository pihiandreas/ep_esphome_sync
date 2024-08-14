#include "ade7953_base.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace ade7953_base {

static const char *const TAG = "ade7953";

// from https://github.com/arendst/Tasmota/blob/development/tasmota/tasmota_xnrg_energy/xnrg_07_ade7953.ino
// #define ADE7953_PREF              1540       // 4194304 / (1540 / 1000) = 2723574 (= WGAIN, VAGAIN and VARGAIN)
// #define ADE7953_UREF              26000      // 4194304 / (26000 / 10000) = 1613194 (= VGAIN)
// #define ADE7953_IREF              10000      // 4194304 / (10000 / 10000) = 4194303 (= IGAIN, needs to be different than 4194304 in order to use calib.dat)
// #define ADE7953_NO_LOAD_THRESHOLD 29196      // According to ADE7953 datasheet the default threshold for no load detection is 58,393 use half this value to measure lower (5w) powers.
static const float ADE7953_PREF = 154.0f;
static const float ADE7953_UREF = 26000.0f;
static const float ADE7953_IREF = 100000.0f;
static const float ADE7953_WATTSEC_PREF = ADE7953_PREF * ADE7953_PREF / 3600.0f; //  = 6,58777778
static const uint16_t ADE7953_NO_LOAD_THRESHOLD = 29196;

// Esphome 'active power' calculation from 'active energy' 0x31E/0x31F:
// POW(Watt) = READVALUE (int32_t -> float) / (ADE7953_WATTSEC_PREF * time_diff_seconds) => 322W = 4240 / (6,58777778 * 2) =  
// Getting negative value on Shelly2PM+_v0.1.9 2022-6-2:
// [VV][i2c.idf:197]: 0x38 TX 031E
// [VV][i2c.idf:173]: 0x38 RX FFFFEF4F
// [D][ade7953:169]: diff = 2011 
// [D][ade7953:170]: pref = 13.248020
// [D][ade7953:171]: aenergya[0x031E] =  -4273.0000 (2.011sec)
// [D][ade7953:172]: pow a =  -322.5388 W

// Tasmota 'active power' calculation:
// const float ADE7953_LSB_PER_WATTSECOND = 2.5;
// const float ADE7953_POWER_CORRECTION = 23.41494;  // See https://github.com/arendst/Tasmota/pull/16941
// uint32_t active_power[ADE7953_MAX_CHANNEL] = { 0, 0 };
// int32_t Ade7953Read(uint16_t reg) {
// .. response = response << 8 | Wire.read();    // receive DATA (MSB first) ..
// return response; }
// EnergySetCalibration(ENERGY_POWER_CALIBRATION, ADE7953_PREF, i); => 1540
// float power_calibration = (float)EnergyGetCalibration(ENERGY_POWER_CALIBRATION, channel) / 10; => 154.0f
// power_calibration /= ADE7953_POWER_CORRECTION; => 154.0 / 23.41494 = 6.57699742
// divider = (Ade7953.calib_data[channel][ADE7953_CAL_WGAIN] != ADE7953_GAIN_DEFAULT) ? ADE7953_LSB_PER_WATTSECOND : power_calibration; => power_calibration
// Energy->active_power[channel] = (float)Ade7953.active_power[channel] / divider;
// e.g. FFFFEF4F => (-4273 / 2.011) / 6.57699742 = 323.0674 W

void ADE7953::setup() {
  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
  }

  // The chip might take up to 100ms to initialise
  this->set_timeout(100, [this]() {
    // this->write_u8_register16_(0x0010, 0x04);
    this->write_u8_register16_(0x00FE, 0xAD);    // Unlock
    this->write_u16_register16_(0x0120, 0x0030); // see: ADE7953 Data Sheet Rev. C | Page 18 of 72 | ADE7953 POWER-UP PROCEDURE
    // Setup LINE CYCLE ACCUMULATION MODE
    // 1. PFMODE (bit 3) = 1 in CONFIG (0x102)
    // 0b 10000000 00000100 = 0x8004 = default
    // 0b 10000000 00001100 = 0x800C = default + bit3 
    this->write_u16_register16_(0x0102, 0x800C);
    // this->write_u16_register16_(0x0102, 0x8004);
    // 2. Enable line cycle accumulation mode, xLWATT and xLVA to 1 on LCYCMODE (0x004)
    // 0b01000000 = 0x40 = default
    // 0b01111111 = 0x7F = enabled on both channels for xLWATT, xLVA and xLVAR
    this->write_u8_register16_(0x0004, 0x7F);
    // this->write_u8_register16_(0x0004, 0x40);
    
    // Setup no load detection and thresholds
    this->write_u32_register16_(0x0001, 0x07);                       // ADE7953_DISNOLOAD on, Disable no load detection, required before setting thresholds
    // this->write_u32_register16_(0x0303, ADE7953_NO_LOAD_THRESHOLD);  // AP_NOLOAD, Set no load treshold for active power, default: 0x00E419 (58393)
    this->write_u32_register16_(0x0303, ADE7953_NO_LOAD_THRESHOLD);  // AP_NOLOAD, Set default: 0x00E419 (58393)
    // this->write_u32_register16_(0x0304, ADE7953_NO_LOAD_THRESHOLD);  // VAR_NOLOAD, Set no load treshold for reactive power, default: 0x00E419 (58393)
    this->write_u32_register16_(0x0304, ADE7953_NO_LOAD_THRESHOLD);  // VAR_NOLOAD, Set default: 0x00E419 (58393)
    // this->write_u32_register16_(0x0305, 0x0);                        // VA_NOLOAD, Set no load treshold for apparent power, default: 0x000000
    // this->write_u32_register16_(0x0001, 0x0);                        // ADE7953_DISNOLOAD off, Enable no load detection

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
    // this->write_s32_register16_(0x0386, 0xF7D6); // AIRMSOS
    // this->write_s32_register16_(0x0392, 0xF7D6); // BIRMSOS

    // Read back gains for debugging
    pga_v_ = this->read_u8_register16_(PGA_V_8);
    pga_ia_ = this->read_u8_register16_(PGA_IA_8);
    pga_ib_ = this->read_u8_register16_(PGA_IB_8);
    vgain_ = this->read_u32_register16_(AVGAIN_32);
    aigain_ = this->read_u32_register16_(AIGAIN_32);
    bigain_ = this->read_u32_register16_(BIGAIN_32);
    awgain_ = this->read_u32_register16_(AWGAIN_32);
    bwgain_ = this->read_u32_register16_(BWGAIN_32);
    ap_noload_ = this->read_u32_register16_(0x0303);
    var_noload_ = this->read_u32_register16_(0x0304);
    va_noload_ = this->read_u32_register16_(0x0305);
    config_ = this->read_u16_register16_(0x0102);
    lcycmode_ = this->read_u8_register16_(0x0004);
    accmode_ = this->read_u32_register16_(0x0301); // The ACCMODE register (Address 0x201 and Address 0x301) includes two sign indication bits that show the sign of the active power of Current Channel A (APSIGN_A) and Current Channel B (APSIGN_B).
    // initial log after boot
    // ACCMODE_32: 0x002D1000 => 00000000001011010001000000000000
    // ACCMODE_32: 0x002D3800 => 00000000001011010011100000000000
    //                           10987654321098765432109876543210
    //                            3         2         1         0
    // 1:0  = 00 = AWATTACC => set to 
    // 3:2  = 00 = BWATTACC
    // 5:4  = 00 = AVARACC
    // 7:6  = 00 = BVARACC
    // 8    = 0  = AVAACC
    // 9    = 0  = BVAACC
    // 10   = 0  = APSIGN_A
    // 11   = 1  = APSIGN_B
    // 12   = 1  = VARSIGN_A
    // 13   = 1  = VARSIGN_B
    // 15:14= 00 = Reserved
    // 16   = 1  = ACTNLOAD_A
    // 17   = 0  = VANLOAD_A
    // 18   = 1  = VARNLOAD_A
    // 19   = 1  = ACTNLOAD_B
    // 20   = 0  = VANLOAD_B
    // 21   = 1  = VARNLOAD_B
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
  // ESP_LOGCONFIG(TAG, "  USE_ACC_ENERGY_REGS: %d", this->use_acc_energy_regs_);
  ESP_LOGCONFIG(TAG, "  PGA_V_8: 0x%X", pga_v_);
  ESP_LOGCONFIG(TAG, "  PGA_IA_8: 0x%X", pga_ia_);
  ESP_LOGCONFIG(TAG, "  PGA_IB_8: 0x%X", pga_ib_);
  ESP_LOGCONFIG(TAG, "  VGAIN_32: 0x%08jX", (uintmax_t) vgain_);
  ESP_LOGCONFIG(TAG, "  AIGAIN_32: 0x%08jX", (uintmax_t) aigain_);
  ESP_LOGCONFIG(TAG, "  BIGAIN_32: 0x%08jX", (uintmax_t) bigain_);
  ESP_LOGCONFIG(TAG, "  AWGAIN_32: 0x%08jX", (uintmax_t) awgain_);
  ESP_LOGCONFIG(TAG, "  BWGAIN_32: 0x%08jX", (uintmax_t) bwgain_);
  ESP_LOGCONFIG(TAG, "  ACCMODE_32: 0x%08jX", (uintmax_t) accmode_);
  ESP_LOGCONFIG(TAG, "  AP_NOLOAD_32: 0x%08jX", (uintmax_t) ap_noload_);
  ESP_LOGCONFIG(TAG, "  VAR_NOLOAD_32: 0x%08jX", (uintmax_t) var_noload_);
  ESP_LOGCONFIG(TAG, "  VA_NOLOAD_32: 0x%08jX", (uintmax_t) va_noload_);
  ESP_LOGCONFIG(TAG, "  LCYCMODE_8: 0x%X", lcycmode_);
  ESP_LOGCONFIG(TAG, "  CONFIG_8: 0x%X", config_);
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

  // Active power & Forward active energy (both from 0x031E / 0x031F)
  const uint32_t now = millis();
  const auto diff = now - this->last_update_;
  this->last_update_ = now;
  // prevent DIV/0
  float pref = ADE7953_WATTSEC_PREF * (diff < 10 ? 10 : diff) / 1000.0f;
  float eref = ADE7953_WATTSEC_PREF * 3600.0f; // to Wh
  
  float aenergya = this->read_s32_register16_(0x031E);
  ESP_LOGD(TAG, "diff = %" PRIu32 " ", diff);
  ESP_LOGD(TAG, "pref = %f", pref);
  ESP_LOGD(TAG, "aenergya[0x031E] =  %.4f", aenergya);
  ESP_LOGD(TAG, "pow a =  %.4f W", aenergya / pref);
  this->active_power_a_sensor_->publish_state(aenergya / pref);
  this->forward_active_energy_a_total += (aenergya / eref);
  this->forward_active_energy_a_sensor_->publish_state(this->forward_active_energy_a_total);

  float aenergyb = this->read_s32_register16_(0x031F);
  ESP_LOGD(TAG, "aenergyb[0x031F] =  %.4f", aenergyb);
  ESP_LOGD(TAG, "pow b =  %.4f W", aenergyb / pref);
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
