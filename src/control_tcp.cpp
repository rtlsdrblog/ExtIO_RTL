
#include "control.h"
#include "rates.h"
#include "tuners.h"

#include "LC_ExtIO_Types.h"

#include <stdio.h>
#include <assert.h>
#include <cmath>


#ifdef _MSC_VER
#pragma warning(disable : 4996)
#define snprintf  _snprintf
#endif

#define N_TUNER_RETRIES  1

//                                                       A  B  C  D  E
std::atomic_int GPIO_pin[ControlVars::NUM_GPIO_BUTTONS] = { 0, 1, 2, 4, 5 };
std::atomic_int GPIO_inv[ControlVars::NUM_GPIO_BUTTONS] = { 0, 0, 0, 0, 0 };
std::atomic_int GPIO_en[ControlVars::NUM_GPIO_BUTTONS] = { 1, 1, 1, 1, 1 };
char GPIO_txt[ControlVars::NUM_GPIO_BUTTONS][16] = {
    "! 4.5V BIAS T !",
    "GPIO1 / PIN32",
    "GPIO2 / PIN31",
    "GPIO4 / PIN30",
    "GPIO5 / PIN29"
};


ControlVars last{ false }; // init_next = false;
ControlVars nxt{ true };   // init_next = true;
std::atomic<CtrlFlagT> somewhat_changed = 0;
std::atomic_bool commandEverything = true;

const int* bandwidths = 0;
const int* rf_gains = 0;
const int* if_gains = 0;
int n_bandwidths = 0;   // tuner_a_bws[]
int n_rf_gains = 0;     // tuners::rf_gains[]
int n_if_gains = 0;     // tuner_a_if_gains[]

bool RtlDeviceInfo::is_same(const RtlDeviceInfo& A, const RtlDeviceInfo& B)
{
  int r = memcmp(A.vendor, B.vendor, sizeof(vendor));
  if (r)
    return false;
  r = memcmp(A.product, B.product, sizeof(product));
  if (r)
    return false;
  r = memcmp(A.serial, B.serial, sizeof(serial));
  if (r)
    return false;
  return true;
}


RtlDeviceInfo RtlDeviceList[MAX_RTL_DEVICES];
RtlDeviceInfo RtlOpenDevice;
uint32_t RtlNumDevices = 0;
uint32_t RtlSelectedDeviceIdx = 0;

rtlsdr_dev_t* RtlSdrDev = 0;
std::atomic_uint32_t tunerNo = RTLSDR_TUNER_UNKNOWN;
std::atomic_bool GotTunerInfo = false;

/* ExtIO Callback */
extern pfnExtIOCallback gpfnExtIOCallbackPtr;

// error message, with "const char*" in IQdata,
//   intended for a log file  AND  a message box
#define SDRLOG( A, TEXT ) do { if ( gpfnExtIOCallbackPtr ) gpfnExtIOCallbackPtr(-1, A, 0, TEXT ); } while (0)

#define SDRLG( A, TEXT, ...) do { \
  if ( gpfnExtIOCallbackPtr ) { \
    snprintf(acMsg, 255, TEXT, __VA_ARGS__); \
    acMsg[255] = 0; \
    gpfnExtIOCallbackPtr(-1, A, 0, acMsg ); \
  } \
} while (0)


static constexpr unsigned NUM_GPIO_BUTTONS = ControlVars::NUM_GPIO_BUTTONS;


static inline void clear_flag(CtrlFlagT& flags, CtrlFlagT f)
{
  flags &= ~f;
}


static inline bool isR82XX()
{
  const int t = tunerNo;
  return (RTLSDR_TUNER_R820T == t || RTLSDR_TUNER_R828D == t || RTLSDR_TUNER_BLOG_V4 == t || RTLSDR_TUNER_BLOG_V4L == t);
}

int nearestBwIdx(int bw)
{
  if (bw <= 0 || n_bandwidths <= 0)
    return 0;
  else if (bw <= bandwidths[1])
    return 1;
  else if (bw >= bandwidths[n_bandwidths - 1])
    return n_bandwidths - 1;

  int nearest_idx = 1;
  int nearest_dist = 10000000;
  for (int idx = 1; idx < n_bandwidths; ++idx)
  {
    int dist = std::abs(bw - bandwidths[idx]);
    if (dist < nearest_dist)
    {
      nearest_idx = idx;
      nearest_dist = dist;
    }
  }
  return nearest_idx;
}

int nearestGainIdx(int gain, const int* gains, const int n_gains)
{
  if (n_gains <= 0)
    return 0;
  else if (gain <= gains[0])
    return 0;
  else if (gain >= gains[n_gains - 1])
    return n_gains - 1;

  int nearest_idx = 0;
  int nearest_dist = 10000000;
  for (int idx = 0; idx < n_gains; ++idx)
  {
    int dist = abs(gain - gains[idx]);
    if (dist < nearest_dist)
    {
      nearest_idx = idx;
      nearest_dist = dist;
    }
  }
  return nearest_idx;
}


uint32_t retrieve_devices()
{
  // const uint32_t prevRtlNumDevices = RtlNumDevices;
  RtlNumDevices = 0;
  bool replaced_open_dev = false;
  uint32_t N = rtlsdr_get_device_count();
  if (N > MAX_RTL_DEVICES)
    N = MAX_RTL_DEVICES;

  for (uint32_t k = 0; k < N; ++k)
  {
    RtlDeviceInfo& dev_info = RtlDeviceList[RtlNumDevices];
    dev_info.clear();

    int r = rtlsdr_get_device_usb_strings(k, dev_info.vendor, dev_info.product, dev_info.serial);
    if (r < 0)
    {
      if (RtlSdrDev && !replaced_open_dev)
      {
        // int r = rtlsdr_get_usb_strings(RtlSdrDev, dev_info.vendor, dev_info.product, dev_info.serial);
        dev_info = RtlOpenDevice;
        snprintf(dev_info.name, 255, "%s / %s / %s (*)",
          RtlOpenDevice.vendor,
          RtlOpenDevice.product,
          RtlOpenDevice.serial);
        dev_info.name[255] = 0;
        replaced_open_dev = true;
      }
      else
        continue;
    }
    else
    {
      dev_info.dev_idx = k;

      // create 'name'
      snprintf(dev_info.name, 255, "%s / %s / %s",
        dev_info.vendor,
        dev_info.product,
        dev_info.serial);
      dev_info.name[255] = 0;
    }
    ++RtlNumDevices;
  }

  if (!RtlNumDevices)
  {
    RtlDeviceInfo& dev_info = RtlDeviceList[RtlNumDevices];
    dev_info.clear();
    snprintf(dev_info.name, 255, "%s", "No compatible device found!");
    dev_info.name[255] = 0;
    dev_info.dev_idx = MAX_RTL_DEVICES;
    ++RtlNumDevices;
  }

  if (RtlSelectedDeviceIdx >= RtlNumDevices)
    RtlSelectedDeviceIdx = 0;

  char acMsg[256];
  SDRLG(extHw_MSG_DEBUG, "retrieve_devices(): found %u devices:", RtlNumDevices);
  for (unsigned k = 0; k < RtlNumDevices; ++k)
  {
    RtlDeviceInfo& dev_info = RtlDeviceList[k];
    SDRLG(extHw_MSG_DEBUG, "retrieve_devices(): dev %u: %s", unsigned(dev_info.dev_idx), dev_info.name);
  }

  return RtlNumDevices;
}

bool is_device_handle_valid()
{
  char acMsg[256];
  if (!RtlSdrDev)
  {
    SDRLOG(extHw_MSG_WARNING, "is_device_handle_valid(): invalid handle!");
    return false;
  }

  int r = rtlsdr_is_connected(RtlSdrDev, 100);
  if (r < 0) {
    SDRLG(extHw_MSG_ERROR, "is_device_handle_valid(): handle 0x%p invalid!", RtlSdrDev);
    return false;
  }

  SDRLG(extHw_MSG_DEBUG, "is_device_handle_valid(): handle 0x%p is ok.", RtlSdrDev);
  return true;
}

void close_rtl_device()
{
  char acMsg[256];
  if (RtlSdrDev)
    SDRLG(extHw_MSG_DEBUG, "close_rtl_device(handle 0x%p)", RtlSdrDev);
  rtlsdr_close(RtlSdrDev);
  RtlSdrDev = 0;
  tunerNo = RTLSDR_TUNER_UNKNOWN;
  GotTunerInfo = false;
  RtlOpenDevice.clear();
}

bool open_selected_rtl_device()
{
  char acMsg[256];
  close_rtl_device();

  if (RtlSelectedDeviceIdx >= MAX_RTL_DEVICES
    || RtlDeviceList[RtlSelectedDeviceIdx].dev_idx >= MAX_RTL_DEVICES)
    return false;

  RtlOpenDevice = RtlDeviceList[RtlSelectedDeviceIdx];
  SDRLG(extHw_MSG_DEBUG, "opening RTL device %u idx %u: %s",
    unsigned(RtlOpenDevice.dev_idx), unsigned(RtlSelectedDeviceIdx), RtlOpenDevice.name);
  int r = rtlsdr_open(&RtlSdrDev, RtlOpenDevice.dev_idx);
  if (r < 0)
  {
    SDRLG(extHw_MSG_ERROR, "opening RTL device failed: %d", r);
    RtlOpenDevice.clear();
    return false;
  }
  SDRLG(extHw_MSG_DEBUG, "open_selected_rtl_device() -> handle 0x%p", RtlSdrDev);

  rtlsdr_tuner t = rtlsdr_get_tuner_type(RtlSdrDev);
  if (tunerNo < tuners::N)
    SDRLG(extHw_MSG_DEBUG, "opened RTL device has tuner type %s", tuners::names[unsigned(t)]);
  else
    SDRLG(extHw_MSG_ERROR, "opened RTL device has unknown tuner type %u", unsigned(t));

  tunerNo = uint32_t(t);
  GotTunerInfo = true;

  // update bandwidths
  bandwidths = tuners::bws[tunerNo].bw;
  n_bandwidths = tuners::bws[tunerNo].num;
  if (n_bandwidths)
  {
    int bwIdx = nearestBwIdx(nxt.tuner_bw);
    nxt.tuner_bw = bandwidths[bwIdx];
    last.tuner_bw = nxt.tuner_bw + 1;
  }

  // update hf gains
  rf_gains = tuners::rf_gains[tunerNo].gain;
  n_rf_gains = tuners::rf_gains[tunerNo].num;
  if (n_rf_gains)
  {
    int gainIdx = nearestGainIdx(nxt.rf_gain, rf_gains, n_rf_gains);
    nxt.rf_gain = rf_gains[gainIdx];
    last.rf_gain = nxt.rf_gain + 10;
  }

  // update if gains
  if_gains = tuners::if_gains[tunerNo].gain;
  n_if_gains = tuners::if_gains[tunerNo].num;
  if (n_if_gains)
  {
    nxt.if_gain_idx = nearestGainIdx(nxt.if_gain_val, if_gains, n_if_gains);
    nxt.if_gain_val = if_gains[nxt.if_gain_idx];
    last.if_gain_val = nxt.if_gain_val + 10;
    last.if_gain_idx = nxt.if_gain_idx + 1;
  }

  commandEverything.store(true);
  Control_Changes();
  return GotTunerInfo;
}


bool Control_Changes()
{
  char acMsg[256];
  rtlsdr_dev_t* dev = RtlSdrDev;
  if (!dev)
    return false;

  CtrlFlagT changed = somewhat_changed.exchange(0);
  const bool command_all = commandEverything.exchange(false) || (changed & CtrlFlags::everything);

  SDRLG(extHw_MSG_DEBUG, "Control_Changes(): %s changes 0x%x", command_all ? "ALL" : "", unsigned(changed));

  if (last.sampling_mode != nxt.sampling_mode || command_all)
  {
    int tmp = nxt.sampling_mode;
    // printf("set direct sampling %u (=%s)\n", tmp, (!tmp) ? "disabled" : (tmp == 1) ? "pin I-ADC" : (tmp == 2) ? "pin Q-ADC" : "unknown!");
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_direct_sampling()");
    int r = rtlsdr_set_direct_sampling(dev, tmp);
    if (r < 0)
      SDRLG(extHw_MSG_WARNING, "Error setting rtlsdr_set_direct_sampling(): %d", r);
    last.sampling_mode = tmp;
    clear_flag(changed, CtrlFlags::sampling_mode);
  }
  if (last.offset_tuning != nxt.offset_tuning || command_all)
  {
    int tmp = nxt.offset_tuning;
    if (isR82XX())
    {
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_offset_tuning(): ignored for tuner");
    }
    else
    {
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_offset_tuning()");
      int r = rtlsdr_set_offset_tuning(dev, tmp);
      if (r < 0)
        SDRLG(extHw_MSG_WARNING, "Error setting rtlsdr_set_offset_tuning(): %d", r);
    }
    last.offset_tuning = tmp;
    clear_flag(changed, CtrlFlags::offset_tuning);
  }
  if (last.USB_sideband != nxt.USB_sideband || command_all)
  {
    int tmp = nxt.USB_sideband.load() ? 1 : 0;
    // printf("set tuner sideband %d: %s sideband\n", tmp, (tmp ? "upper" : "lower"));
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_sideband()");
    int r = -1;
    for (int retry = 0; r < 0 && retry < N_TUNER_RETRIES; ++retry)  // retry!?!?!
      r = rtlsdr_set_tuner_sideband(dev, tmp);
    if (r < 0)
      SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_sideband(): %d", r);
    else
      last.USB_sideband = tmp;
    clear_flag(changed, CtrlFlags::tuner_sideband);
  }
  for (int btnNo = 0; btnNo < NUM_GPIO_BUTTONS; ++btnNo)
  {
    if (GPIO_en[btnNo] && (last.GPIO[btnNo] != nxt.GPIO[btnNo] || command_all))
    {
      int r, tmp = nxt.GPIO[btnNo];
      const int GPIOpin = GPIO_pin[btnNo];
      const int GPIOval = tmp ^ GPIO_inv[btnNo];
      if (GPIOpin < 0)
      {
        // printf("set bias T %u (%s)\n", tmp, tmp ? "on" : "off");
        SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_bias_tee()");
        r = rtlsdr_set_bias_tee(dev, GPIOval);  // SET_BIAS_TEE
      }
      else
      {
        // transmitTcpCmd(conn, GPIO_WRITE_PIN, (GPIOpin << 16) | GPIOval); // GPIO_WRITE_PIN
        // printf("write %d to gpio %d\n", itmp & 0xffff, (itmp >> 16) & 0xffff);
        SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_gpio_output() / rtlsdr_set_gpio_bit()");
        r = rtlsdr_set_gpio_output(dev, uint8_t(GPIOpin));
        r = rtlsdr_set_gpio_bit(dev, uint8_t(GPIOpin), GPIOval);
      }
      last.GPIO[btnNo] = tmp;
    }
    clear_flag(changed, CtrlFlags::gpio);
  }
  if (last.freq_corr_ppm != nxt.freq_corr_ppm || command_all)
  {
    int tmp = nxt.freq_corr_ppm;
    // printf("set freq correction %d ppm\n", itmp);
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_freq_correction()");
    int r = 0;
    int curr_ppm = rtlsdr_get_freq_correction(dev);
    if (curr_ppm != tmp)
    {
      r = rtlsdr_set_freq_correction(dev, tmp);
      if (r < 0)
        SDRLG(extHw_MSG_WARNING, "Error setting rtlsdr_set_freq_correction(): %d", r);
    }
    last.freq_corr_ppm = tmp;
    clear_flag(changed, CtrlFlags::ppm_correction);
  }
  if (last.band_center_sel != nxt.band_center_sel || command_all)
  {
    int fs = rates::tab[nxt.srate_idx].valueInt;
    int tmp = nxt.band_center_sel;
    int band_center = 0;
    if (tmp == 1)
      band_center = fs / 4;
    else if (tmp == 2)
      band_center = -fs / 4;

    // printf("set tuner band to IF frequency %i Hz from center\n", if_band_center_freq);
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_band_center()");
    int r = rtlsdr_set_tuner_band_center(dev, band_center);
    if (r < 0)
      SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_band_center(): %d", r);
    else
    {
      last.band_center_sel = tmp;
      last.band_center_LO_delta.store(nxt.band_center_LO_delta.load());
    }
    clear_flag(changed, CtrlFlags::tuner_band_center);
    changed |= CtrlFlags::freq;
  }
  const uint64_t f64 = uint64_t(nxt.LO_freq.load());
  if (last.LO_freq.load() != f64 || (changed & CtrlFlags::freq) || command_all)
  {
    int prev_on, prev_counter;
    rtlsdr_get_impulse_nc(dev, &prev_on, &prev_counter);
    SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_get_impulse_nc() -> %d, %d)", prev_on, prev_counter);

    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_center_freq64()");
    int r = rtlsdr_set_center_freq64(dev, f64);
    if (r < 0)
      SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_center_freq64(): %d", r);
    else
      last.LO_freq.store(f64);
    clear_flag(changed, CtrlFlags::freq);
  }
  if (last.srate_idx != nxt.srate_idx || command_all)
  {
    // re-parametrize Tuner RF AGC
    {
      int tmp = nxt.tuner_rf_agc;
      // transmitTcpCmd(conn, SET_GAIN_MODE, 1 - tmp);
      // printf("set gain mode %u (=%s)\n", tmp, tmp ? "manual" : "automatic");
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_gain_mode()");
      int r = rtlsdr_set_tuner_gain_mode(dev, 1 - tmp);
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_gain_mode(): %d", r);
      else
        last.tuner_rf_agc = tmp;
      clear_flag(changed, CtrlFlags::rf_agc);
    }

    // re-parametrize Gain
    if (nxt.tuner_rf_agc == 0)
    {
      int tmp = nxt.rf_gain;
      // printf("set manual tuner gain %.1f dB\n", tmp / 10.0);
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_gain()");
      int r = rtlsdr_set_tuner_gain(dev, tmp);  // SET_GAIN
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_gain(): %d", r);
      else
        last.rf_gain = tmp;
      clear_flag(changed, CtrlFlags::rf_gain);
    }

    // re-parametrize Tuner IF AGC and/or IF gain
    if (!isR82XX())
    {
    }
    else if (nxt.tuner_if_agc)
    {
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_if_mode(0)");
      int r = rtlsdr_set_tuner_if_mode(dev, 0);  // SET_TUNER_IF_MODE; 0 activates AGC
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_if_mode(): %d", r);
      else
      {
        last.tuner_if_agc.store(1);
        last.if_gain_idx.store(nxt.if_gain_idx.load());
      }
      clear_flag(changed, CtrlFlags::if_agc_gain);
    }
    else
    {
      int tmp = nxt.if_gain_idx.load();
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_if_mode(10000+)");
      int r = rtlsdr_set_tuner_if_mode(dev, 10000 + tmp);  // SET_TUNER_IF_MODE
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_if_mode(): %d", r);
      else
      {
        last.tuner_if_agc.store(0);
        last.if_gain_idx.store(tmp);
      }
      clear_flag(changed, CtrlFlags::if_agc_gain);
    }

    // re-parametrize Tuner Bandwidth
    {
      int tmp = nxt.tuner_bw;
      uint32_t applied_bw = 0;
      if (n_bandwidths)
      {
        // SET_TUNER_BANDWIDTH
        SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_and_get_tuner_bandwidth()");
        rtlsdr_set_and_get_tuner_bandwidth(dev, tmp * 1000, &applied_bw, 1 /* =apply_bw */);
      }
      last.tuner_bw = tmp;
      clear_flag(changed, CtrlFlags::tuner_bandwidth);
    }

    // re-parametrize samplerate
    {
      int tmp = nxt.srate_idx;
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_sample_rate()");
      int r = rtlsdr_set_sample_rate(dev, rates::tab[tmp].valueInt);  // SET_SAMPLE_RATE
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_sample_rate(): %d", r);
      else
        last.srate_idx = tmp;
      clear_flag(changed, CtrlFlags::srate);
    }

    if (last.band_center_sel != nxt.band_center_sel || command_all)
    {
      int tmp_srate_idx = nxt.srate_idx;
      int tmp_bcsel = nxt.band_center_sel;
      int fs = rates::tab[tmp_srate_idx].valueInt;
      int band_center = 0;
      if (tmp_bcsel == 1)
        band_center = fs / 4;
      else if (tmp_bcsel == 2)
        band_center = -fs / 4;

      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_band_center()");
      int r = rtlsdr_set_tuner_band_center(dev, band_center);  // SET_TUNER_BW_IF_CENTER
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_band_center(): %d", r);
      else
        last.band_center_sel = tmp_bcsel;
      clear_flag(changed, CtrlFlags::tuner_band_center);
    }
  }

  if (last.tuner_bw != nxt.tuner_bw)
  {
    //if (!transmitTcpCmd(conn, 0x0E, nxt.tunerBW*1000))
    //  return false;
    int tmp = nxt.tuner_bw;
    uint32_t applied_bw = 0;  // SET_TUNER_BANDWIDTH
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_and_get_tuner_bandwidth()");
    int r = 1;
    for (int retry = 0; r && retry < N_TUNER_RETRIES; ++retry )  // retry!?!?!
      r = rtlsdr_set_and_get_tuner_bandwidth(dev, tmp * 1000, &applied_bw, 1 /* =apply_bw */);
    SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_and_get_tuner_bandwidth(%d) -> bw %u, rc %d",
      tmp * 1000, unsigned(applied_bw), r);
    last.tuner_bw = tmp;
    clear_flag(changed, CtrlFlags::tuner_bandwidth);
  }

  if (last.tuner_rf_agc != nxt.tuner_rf_agc)
  {
    int tmp = nxt.tuner_rf_agc;
    int tmp_gain = nxt.rf_gain;
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_gain_mode()");
    int r = rtlsdr_set_tuner_gain_mode(dev, 1 - tmp);
    if (r < 0)
      SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_gain_mode(): %d", r);
    else
    {
      last.tuner_rf_agc = tmp;
      if (tmp == 0)
        last.rf_gain = tmp_gain + 1;
    }
    clear_flag(changed, CtrlFlags::rf_agc);
  }

  if (last.rtl_agc != nxt.rtl_agc || command_all)
  {
    int tmp = nxt.rtl_agc;
    // printf("set rtl2832's digital agc mode %d (=%s)\n", tmp, tmp ? "enabled" : "disabled");
    SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_agc_mode()");
    int r = rtlsdr_set_agc_mode(dev, tmp);  // SET_AGC_MODE
    if (r < 0)
      SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_agc_mode(): %d", r);
    else
      last.rtl_agc = tmp;
    clear_flag(changed, CtrlFlags::rtl_agc);
  }

  if (last.rf_gain != nxt.rf_gain)
  {
    int tmp = nxt.rf_gain;
    if (nxt.tuner_rf_agc == 0)
    {
      // transmit manual gain only when TunerAGC is off
      // printf("set manual tuner gain %.1f dB\n", tmp / 10.0);
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_gain()");
      int r = rtlsdr_set_tuner_gain(dev, tmp);  // SET_GAIN
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_gain(): %d", r);
      else
        last.rf_gain = tmp;
    }
    clear_flag(changed, CtrlFlags::rf_gain);
  }

  if (last.tuner_if_agc != nxt.tuner_if_agc || last.if_gain_idx != nxt.if_gain_idx)
  {
    int tmp_agc = nxt.tuner_if_agc;
    int tmp_gain = nxt.if_gain_idx;
    if (!isR82XX())
    {

    }
    else if (tmp_agc)
    {
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_if_mode()");
      int r = rtlsdr_set_tuner_if_mode(dev, 0);  // SET_TUNER_IF_MODE
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_if_mode(): %d", r);
      else
      {
        last.tuner_if_agc.store(tmp_agc);
        last.if_gain_idx.store(tmp_gain);
      }
      clear_flag(changed, CtrlFlags::if_agc_gain);
    }
    else
    {
      SDRLOG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_tuner_if_mode()");
      int r = rtlsdr_set_tuner_if_mode(dev, 10000 + tmp_gain);  // SET_TUNER_IF_MODE
      if (r < 0)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_tuner_if_mode(): %d", r);
      else
      {
        last.tuner_if_agc = tmp_agc;
        last.if_gain_idx = tmp_gain;
      }
      clear_flag(changed, CtrlFlags::if_agc_gain);
    }
  }

  if (last.rtl_impulse_noise_cancellation != nxt.rtl_impulse_noise_cancellation
    || (changed & CtrlFlags::rtl_impulse_nc) || command_all)
  {
    int tmp = nxt.rtl_impulse_noise_cancellation;
    if (tmp == 0 || tmp == 1)
    {
      int prev_on, prev_counter;
      rtlsdr_get_impulse_nc(dev, &prev_on, &prev_counter);
      SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_get_impulse_nc() -> %d, %d", prev_on, prev_counter);

      SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_impulse_nc(%d)", tmp);
      int r = rtlsdr_set_impulse_nc(dev, tmp, tmp);
      if (r)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_impulse_nc(): %d", r);
    }
    last.rtl_impulse_noise_cancellation = tmp;
    clear_flag(changed, CtrlFlags::rtl_impulse_nc);
  }

  if (
    last.rtl_aagc_rf_en != nxt.rtl_aagc_rf_en
    || last.rtl_aagc_rf_inv != nxt.rtl_aagc_rf_inv
    || last.rtl_aagc_rf_min != nxt.rtl_aagc_rf_min
    || last.rtl_aagc_rf_max != nxt.rtl_aagc_rf_max
    || last.rtl_aagc_if_en != nxt.rtl_aagc_if_en
    || last.rtl_aagc_if_inv != nxt.rtl_aagc_if_inv
    || last.rtl_aagc_if_min != nxt.rtl_aagc_if_min
    || last.rtl_aagc_if_max != nxt.rtl_aagc_if_max
    || last.rtl_aagc_lg_lock != nxt.rtl_aagc_lg_lock
    || last.rtl_aagc_lg_unlock != nxt.rtl_aagc_lg_unlock
    || last.rtl_aagc_lg_ifr != nxt.rtl_aagc_lg_ifr
    || command_all)
  {
    unsigned tRfEn = nxt.rtl_aagc_rf_en;
    unsigned tRfInv = nxt.rtl_aagc_rf_inv;
    unsigned tRfMin = nxt.rtl_aagc_rf_min;
    unsigned tRfMax = nxt.rtl_aagc_rf_max;
    unsigned tIfEn = nxt.rtl_aagc_if_en;
    unsigned tIfInv = nxt.rtl_aagc_if_inv;
    unsigned tIfMin = nxt.rtl_aagc_if_min;
    unsigned tIfMax = nxt.rtl_aagc_if_max;
    unsigned tLGLck = nxt.rtl_aagc_lg_lock;
    unsigned tLGUck = nxt.rtl_aagc_lg_unlock;
    unsigned tLGIfr = nxt.rtl_aagc_lg_ifr;
    {
      int prev_en_rf, prev_inv_rf, prev_rf_min, prev_rf_max;
      int prev_en_if, prev_inv_if, prev_if_min, prev_if_max;
      int prev_gain_lock, prev_gain_unlock, prev_gain_interference;
      rtlsdr_get_aagc(dev,
        &prev_en_rf, &prev_inv_rf, &prev_rf_min, &prev_rf_max,
        &prev_en_if, &prev_inv_if, &prev_if_min, &prev_if_max,
        &prev_gain_lock, &prev_gain_unlock, &prev_gain_interference);
      SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_get_aagc()\n"
        "  -> RF: en %d, inv %d, %d - %d\n"
        "  -> IF: en %d, inv %d, %d - %d\n"
        "  -> loop: lock %d, unlock %d, interference %d",
        prev_en_rf, prev_inv_rf, prev_rf_min, prev_rf_max,
        prev_en_if, prev_inv_if, prev_if_min, prev_if_max,
        prev_gain_lock, prev_gain_unlock, prev_gain_interference);

      SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_aagc(\n"
        "  RF: en %d, inv %d, %d - %d,\n"
        "  IF: en %d, inv %d, %d - %d,\n"
        "  loop gain: lock %d, unlock %d, interference %d )",
        tRfEn, tRfInv, tRfMin, tRfMax,
        tIfEn, tIfInv, tIfMin, tIfMax,
        tLGLck, tLGUck, tLGIfr);
      int r = rtlsdr_set_aagc(dev,
        tRfEn, tRfInv, tRfMin, tRfMax,
        tIfEn, tIfInv, tIfMin, tIfMax,
        tLGLck, tLGUck, tLGIfr);
      if (r)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_aagc(): %d", r);
    }
    if (tRfEn < 2)      last.rtl_aagc_rf_en = tRfEn;
    if (tRfInv < 2)     last.rtl_aagc_rf_inv = tRfInv;
    if (tRfMin < 256U)  last.rtl_aagc_rf_min = tRfMin;
    if (tRfMax < 256U)  last.rtl_aagc_rf_max = tRfMax;
    if (tIfEn < 2)      last.rtl_aagc_if_en = tIfEn;
    if (tIfInv < 2)     last.rtl_aagc_if_inv = tIfInv;
    if (tIfMin < 256U)  last.rtl_aagc_if_min = tIfMin;
    if (tIfMax < 256U)  last.rtl_aagc_if_max = tIfMax;
    if (tLGLck < 32U)   last.rtl_aagc_lg_lock = tLGLck;
    if (tLGUck < 32U)   last.rtl_aagc_lg_unlock = tLGUck;
    if (tLGIfr < 32U)   last.rtl_aagc_lg_ifr  = tLGIfr;
  }

  if (last.rtl_aagc_vtop[0] != nxt.rtl_aagc_vtop[0]
    || last.rtl_aagc_vtop[1] != nxt.rtl_aagc_vtop[1]
    || last.rtl_aagc_vtop[2] != nxt.rtl_aagc_vtop[2]
    || last.rtl_aagc_krf[0] != nxt.rtl_aagc_krf[0]
    || last.rtl_aagc_krf[1] != nxt.rtl_aagc_krf[1]
    || last.rtl_aagc_krf[2] != nxt.rtl_aagc_krf[2]
    || last.rtl_aagc_krf[3] != nxt.rtl_aagc_krf[3]
    || command_all)
  {
    int vtop[3] = { nxt.rtl_aagc_vtop[0], nxt.rtl_aagc_vtop[1], nxt.rtl_aagc_vtop[2] };
    int krf[4] = { nxt.rtl_aagc_krf[0], nxt.rtl_aagc_krf[1], nxt.rtl_aagc_krf[2], nxt.rtl_aagc_krf[3] };

    int prev_vtop[3], prev_krf[4];
    rtlsdr_get_aagc_gain_distrib(dev, prev_vtop, prev_krf);
    SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_get_agc_gain_distrib() -> vtop[]: %d, %d, %d  krf[]: %d, %d, %d, %d",
      prev_vtop[0], prev_vtop[1], prev_vtop[2],
      prev_krf[0], prev_krf[1], prev_krf[2], prev_krf[3]);

    SDRLG(extHw_MSG_DEBUG, "Control_Changes(): rtlsdr_set_agc_gain_distrib(vtop[]: %d, %d, %d  krf[]: %d, %d, %d, %d)",
      vtop[0], vtop[1], vtop[2],
      krf[0], krf[1], krf[2], krf[3]);
      int r = rtlsdr_set_aagc_gain_distrib(dev, vtop, krf);
      if (r)
        SDRLG(extHw_MSG_ERROR, "Error setting rtlsdr_set_agc_gain_distrib(): %d", r);

    last.rtl_aagc_vtop[0] = vtop[0];
    last.rtl_aagc_vtop[1] = vtop[1];
    last.rtl_aagc_vtop[2] = vtop[2];
    last.rtl_aagc_krf[0] = krf[0];
    last.rtl_aagc_krf[1] = krf[1];
    last.rtl_aagc_krf[2] = krf[2];
    last.rtl_aagc_krf[3] = krf[3];
  }

  return true;
}
