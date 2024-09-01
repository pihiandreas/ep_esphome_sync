#include "ade7953_base.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace ade7953_base {

static const char *const TAG = "ade7953";

enum ADE7953_VALUES { VRMS, FREQ, IRMSA, IRMSB, AENERGYA, AENERGYB, RENERGYA, RENERGYB, APENERGYA, APENERGYB, PFA, PFB, LAST_IDX };                           // indices for values
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
// [D][ade7953:169]: diff = 2011 ms
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

#ifdef USE_GPTIMER
  gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, // 1MHz, 1 tick=1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &(this->gptimer)));
  ESP_ERROR_CHECK(gptimer_enable(this->gptimer));
  ESP_ERROR_CHECK(gptimer_start(this->gptimer));

#endif // USE_GPTIMER

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
    //this->write_u16_register16_(0x0102, 0x8004);
    // 2. Enable line cycle accumulation mode, xLWATT and xLVA to 1 on LCYCMODE (0x004)
    // 0b01000000 = 0x40 = default
    // 0b01111111 = 0x7F = enabled on both channels for xLWATT, xLVA and xLVAR
    // this->write_u8_register16_(0x0004, 0x7F);
    this->write_u8_register16_(0x0004, 0x40);

    // // Setup no load detection and thresholds
    // this->write_u32_register16_(0x0001, 0x07);                       // ADE7953_DISNOLOAD on, Disable no load detection, required before setting thresholds
    // // this->write_u32_register16_(0x0303, ADE7953_NO_LOAD_THRESHOLD);  // AP_NOLOAD, Set no load treshold for active power, default: 0x00E419 (58393)
    // this->write_u32_register16_(0x0303, 0x00E419);  // AP_NOLOAD, Set default: 0x00E419 (58393)
    // // this->write_u32_register16_(0x0304, ADE7953_NO_LOAD_THRESHOLD);  // VAR_NOLOAD, Set no load treshold for reactive power, default: 0x00E419 (58393)
    // this->write_u32_register16_(0x0304, 0x00E419);  // VAR_NOLOAD, Set default: 0x00E419 (58393)
    // // this->write_u32_register16_(0x0305, 0x0);                        // VA_NOLOAD, Set no load treshold for apparent power, default: 0x000000
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
    this->read_u8_register16_(PGA_V_8, &pga_v_);
    this->read_u8_register16_(PGA_IA_8, &pga_ia_);
    this->read_u8_register16_(PGA_IB_8, &pga_ib_);
    this->read_u32_register16_(AVGAIN_32, &vgain_);
    this->read_u32_register16_(AIGAIN_32, &aigain_);
    this->read_u32_register16_(BIGAIN_32, &bigain_);
    this->read_u32_register16_(AWGAIN_32, &awgain_);
    this->read_u32_register16_(BWGAIN_32, &bwgain_);
    this->read_u32_register16_(0x0303, &ap_noload_);
    this->read_u32_register16_(0x0304, &var_noload_);
    this->read_u32_register16_(0x0305, &va_noload_);
    this->read_u16_register16_(0x0102, &config_);
    this->read_u8_register16_(0x0004, &lcycmode_);
    this->read_u32_register16_(0x0301, &accmode_);
    // The ACCMODE register (Address 0x201 and Address 0x301) includes two sign indication bits that show the sign of the active power of Current Channel A (APSIGN_A) and Current Channel B (APSIGN_B).
    // initial log after boot
    //             0x002D1400    00000000001011010001010000000000
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
    this->last_update_ = timestamp_();
    this->is_setup_ = true;
  });
}

void ADE7953::dump_config() {
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
  LOG_SENSOR("  ", "Active Energy A Sensor", this->active_energy_a_sensor_);
  LOG_SENSOR("  ", "Active Energy B Sensor", this->active_energy_b_sensor_);
  ESP_LOGCONFIG(TAG, "  Invert Active Power A: %d", this->apinva_);
  ESP_LOGCONFIG(TAG, "  Invert Active Power B: %d", this->apinvb_);
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
  ESP_LOGD(TAG, "Debug SPI:");
  uint8_t val8{0};
  this->read_u8_register16_(0x0004, &val8);
  ESP_LOGD(TAG, "  LCYCMODE : 0x0004 = 0x%02X (default: 0x40)", val8);
  uint16_t val16{0};
  this->read_u16_register16_(0x0102, &val16);
  ESP_LOGD(TAG, "  CONFIG   : 0x0102 = 0x%04X (default: 0x8004)", val16);
  uint32_t val32{0};
  this->read_u32_register16_(0x0303, &val32);
  // ESP_LOGD(TAG, "  AP_NOLOAD: 0x0303 = 0x%08" PRIx32 " (default: 0x0000E419)", val32);
  ESP_LOGD(TAG, "  AP_NOLOAD: 0x0303 = 0x%08jX (default: 0x0000E419)", (uintmax_t) val32);
}

uint64_t ADE7953::timestamp_() {
#ifdef USE_GPTIMER
  uint64_t count;
  gptimer_get_raw_count(this->gptimer, &count);
  return count;
#else
  return (uint64_t)micros();
#endif // USE_GPTIMER
}

void ADE7953::get_data_() {

  uint16_t u16 = 0;
  int32_t s32 = 0;

  // Frequency
  this->read_u16_register16_(0x010E, &(this->data_.frequency));

  // Voltage
  this->read_u32_register16_(0x031C, &(this->data_.voltage_rms));

  // Current
  this->read_u32_register16_(0x031A, &(this->data_.current_rms_a));
  this->read_u32_register16_(0x031B, &(this->data_.current_rms_b));

  // Power factor
  this->read_s16_register16_(0x010A, &(this->data_.power_factor_a));
  this->read_s16_register16_(0x010B, &(this->data_.power_factor_b));

  // Record time and diff to last update
  const uint64_t now = timestamp_();
  this->data_.ts_diff = now - this->last_update_;
  this->last_update_ = now;

  // Active power & active energy (both from 0x031E / 0x031F = Active Energy A/B)
  this->read_s32_register16_(0x031E, &(this->data_.active_energy_a));
  this->read_s32_register16_(0x031F, &(this->data_.active_energy_b));

  // Reactive power & reactive energy (both from 0x0320 / 0x0321 = Reactive Energy A/B)
  this->read_s32_register16_(0x0320, &(this->data_.reactive_energy_a));
  this->read_s32_register16_(0x0321, &(this->data_.reactive_energy_b));

  // Apparent power & reactive energy (both from 0x0322 / 0x0323 = Apparent Energy A/B)
  this->read_s32_register16_(0x0322, &(this->data_.apparent_energy_a));
  this->read_s32_register16_(0x0323, &(this->data_.apparent_energy_b));
}



void ADE7953::publish_data_() {

  // // Convert values to floats
  float val[LAST_IDX] = {0.0};
  val[FREQ] = 223750.0f / (1.0f + (float)this->data_.frequency);
  val[VRMS] = ((float)this->data_.voltage_rms) / ADE7953_UREF;
  val[IRMSA] = ((float)this->data_.current_rms_a) / ADE7953_IREF;
  val[IRMSB] = ((float)this->data_.current_rms_b) / ADE7953_IREF;
  val[PFA] = ((float)this->data_.power_factor_a) / (0x7FFF / 100.0f);
  val[PFB] = ((float)this->data_.power_factor_b) / (0x7FFF / 100.0f);
  val[AENERGYA] = (float)this->data_.active_energy_a * (this->apinva_ ? -1.0f : 1.0f);
  val[AENERGYB] = (float)this->data_.active_energy_b * (this->apinva_ ? -1.0f : 1.0f);
  val[RENERGYA] = (float)this->data_.reactive_energy_a * (this->apinva_ ? -1.0f : 1.0f);
  val[RENERGYB] = (float)this->data_.reactive_energy_b * (this->apinva_ ? -1.0f : 1.0f);
  val[APENERGYA] = (float)this->data_.apparent_energy_a * (this->apinva_ ? -1.0f : 1.0f);
  val[APENERGYB] = (float)this->data_.apparent_energy_b * (this->apinva_ ? -1.0f : 1.0f);

  float PREF = ADE7953_WATTSEC_PREF * (this->data_.ts_diff < 10000 ? 10000 : this->data_.ts_diff) / 1000000.0f;
  float EREF = ADE7953_WATTSEC_PREF * 3600.0f; // to Wh

  if (this->frequency_sensor_ != nullptr) this->frequency_sensor_->publish_state(val[FREQ]);
  if (this->voltage_sensor_ != nullptr) this->voltage_sensor_->publish_state(val[VRMS]);
  if (this->current_a_sensor_ != nullptr) this->current_a_sensor_->publish_state(val[IRMSA]);
  if (this->current_b_sensor_ != nullptr) this->current_b_sensor_->publish_state(val[IRMSB]);
  if (this->power_factor_a_sensor_ != nullptr) this->power_factor_a_sensor_->publish_state(val[PFA]);
  if (this->power_factor_b_sensor_ != nullptr) this->power_factor_b_sensor_->publish_state(val[PFA]);
  if (this->active_power_a_sensor_ != nullptr) {
    this->active_power_a_sensor_->publish_state(( abs(val[AENERGYA] / PREF) < 5.0 ) ? 0.0f : (val[AENERGYA] / PREF) ); // publish readings below 5W as 0 and  -0.0W as 0.0W
  }
  if (this->active_power_b_sensor_ != nullptr) {
    this->active_power_b_sensor_->publish_state(( abs(val[AENERGYB] / PREF) < 5.0 ) ? 0.0f : (val[AENERGYB] / PREF) ); // publish readings below 5W as 0 and  -0.0W as 0.0W
  }
  if (this->reactive_power_a_sensor_ != nullptr) {
    this->reactive_power_a_sensor_->publish_state(( abs(val[RENERGYA] / PREF) < 5.0 ) ? 0.0f : (val[RENERGYA] / PREF) ); // publish readings below 5 VAr as 0.0 and  -0.0 VAr as 0.0
  }
  if (this->active_power_b_sensor_ != nullptr) {
    this->active_power_b_sensor_->publish_state(( abs(val[RENERGYB] / PREF) < 5.0 ) ? 0.0f : (val[RENERGYB] / PREF) ); // publish readings below 5 VAr as 0.0 and  -0.0 VAr as 0.0
  }
  if (this->apparent_power_a_sensor_ != nullptr) {
    this->apparent_power_a_sensor_->publish_state(( abs(val[APENERGYA] / PREF) < 5.0 ) ? 0.0f : (val[APENERGYA] / PREF) ); // publish readings below 5 VA as 0.0 and  -0.0 VA as 0.0
  }
  if (this->apparent_power_b_sensor_ != nullptr) {
    this->apparent_power_b_sensor_->publish_state(( abs(val[APENERGYB] / PREF) < 5.0 ) ? 0.0f : (val[APENERGYB] / PREF) ); // publish readings below 5 VA as 0.0 and  -0.0 VA as 0.0
  }
  if (this->active_energy_a_sensor_ != nullptr) {
    this->active_energy_a_total += (val[AENERGYA] / EREF);
    this->active_energy_a_sensor_->publish_state(this->active_energy_a_total);
  }
  if (this->active_energy_b_sensor_ != nullptr) {
    this->active_energy_b_total += (val[AENERGYB] / EREF);
    this->active_energy_b_sensor_->publish_state(this->active_energy_b_total);
  }
}

void ADE7953::update() {
  if (!this->is_setup_)
    return;

  uint64_t t[5] = {0};
  uint8_t ti = 0;
  t[ti++] = timestamp_();
  // ESP_LOGD(TAG, "[%lld] Update loop started", t[0]);

  this->get_data_();
  t[ti++] = timestamp_();

  this->publish_data_();
  t[ti++] = timestamp_();

  ESP_LOGD(TAG, "[%lld] Update loop done in            %5.3f ms", t[ti-1], (float)((t[ti-1] - t[0]) / 1000.0) );
  ESP_LOGD(TAG, "[%lld]   Get data %5.3f ms,   publish %5.3f ms", t[ti-1], (float)((t[1] - t[0]) / 1000.0), (float)((t[2] - t[1]) / 1000.0) );

}

}  // namespace ade7953_base
}  // namespace esphome
