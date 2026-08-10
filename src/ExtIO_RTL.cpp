// SPDX-FileCopyrightText: 2013 José Araújo <josemariaaraujo@gmail.com>
// SPDX-FileCopyrightText: 2020 Jorge Maidana <jorgem.seq@gmail.com>
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// ExtIO_RTL.cpp - ExtIO wrapper for librtlsdr
/*
 * ExtIO wrapper
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tuners.h"
#include "rates.h"
#include "control.h"

#include "LC_ExtIO_Types.h"

#include "gui_dlg.h"

#include "config_file.h"

#define LIBRTL_EXPORTS 1
#include "ExtIO_RTL.h"

#include <atomic>
#include <stdint.h>
#include <stdio.h>

#ifdef HAS_WIN_GUI_DLG
#include <process.h>
#include <tchar.h>
#endif

#include <new>

#define ALWAYS_PCMU8  1
#define ALWAYS_PCM16  0
#define MAX_DECIMATIONS ( ALWAYS_PCMU8 ? 1 : 8 )

int VAR_ALWAYS_PCMU8 = ALWAYS_PCMU8;
int VAR_ALWAYS_PCM16 = ALWAYS_PCM16;

/* 0 == just filter (sum) without decimation
* 1 == do full decimation
*/
#define FULL_DECIMATION   0

#define WITH_AGCS   0

#define SETTINGS_IDENTIFIER "RTL_2023.9-1"


#ifdef _MSC_VER
#pragma warning(disable : 4996)
#define snprintf  _snprintf
#endif


extern "C" int  LIBRTL_API EXTIO_CALL SetAttenuator(int atten_idx);
extern "C" int  LIBRTL_API EXTIO_CALL ExtIoSetMGC(int mgc_idx);



static const int buffer_sizes[] = { //in kBytes
  1, 2, 4, 8, 16, 32, 64, 128, 256
};


#if ALWAYS_PCMU8
extHWtypeT extHWtype = exthwUSBdataU8;  /* ExtIO type 8-bit samples */
#else
extHWtypeT extHWtype = exthwUSBdata16;  /* default ExtIO type 16-bit samples */
#endif

static constexpr unsigned NUM_GPIO_BUTTONS = ControlVars::NUM_GPIO_BUTTONS;


bool SDRsupportsLogging = false;
bool SDRsupportsSamplePCMU8 = false;
bool SDRsupportsSampleFormats = false;


#define MAX_BUFFER_LEN    (256*1024)
#define NUM_BUFFERS_BEFORE_CALLBACK   ( MAX_DECIMATIONS + 1 )

static bool rcvBufsAllocated = false;
static int16_t* pcm16_buf[NUM_BUFFERS_BEFORE_CALLBACK + 1] = { 0 };
static uint8_t* rcvBuf[NUM_BUFFERS_BEFORE_CALLBACK + 1] = { 0 };

static uint32_t ExtIODevIdx = 0;    // id: 08 default: 0
static uint32_t RtlSdrDevCount = 0;
static int RtlSdrPllLocked; // 0 = Locked

std::atomic_int64_t retune_value = 0;
std::atomic_int retune_counter = 0;
std::atomic_bool retune_freq = false;

std::atomic_int bufferSizeIdx = 6;// 64 kBytes
std::atomic_int buffer_len = buffer_sizes[6];

static int HDSDR_AGC = 2;


// Thread handle
std::atomic_bool terminate_RX_Thread = false;
std::atomic_bool terminate_ConnCheck_Thread = false;
std::atomic_bool ThreadStreamToSDR = false;
static bool GUIDebugConnection = false;
static volatile HANDLE RX_thread_handle = INVALID_HANDLE_VALUE;
static volatile HANDLE ConnCheck_thread_handle = INVALID_HANDLE_VALUE;

void RX_ThreadProc(void* param);
int Start_RX_Thread();
int Stop_RX_Thread();

void ConnCheck_ThreadProc(void* param);
int Start_ConnCheck_Thread();
int Stop_ConnCheck_Thread();


/* ExtIO Callback */
pfnExtIOCallback gpfnExtIOCallbackPtr = NULL;

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


static HWND h_dialog = NULL;

static bool isR82XX()
{
  const int t = tunerNo;
  return (RTLSDR_TUNER_R820T == t || RTLSDR_TUNER_R828D == t || RTLSDR_TUNER_BLOG_V4 == t || RTLSDR_TUNER_BLOG_V4L == t);
}

static int nearestSrateIdx(int srate)
{
  if (srate <= 0)
    return 0;
  else if (srate <= rates::tab[1].valueInt)
    return 1;
  else if (srate >= rates::tab[rates::N - 1].valueInt)
    return rates::N - 1;

  int nearest_idx = 1;
  int nearest_dist = 10000000;
  for (int idx = 0; idx < rates::N; ++idx)
  {
    int dist = abs(srate - rates::tab[idx].valueInt);
    if (dist < nearest_dist)
    {
      nearest_idx = idx;
      nearest_dist = dist;
    }
  }
  return nearest_idx;
}


extern "C"
bool  LIBRTL_API EXTIO_CALL InitHW(char* name, char* model, int& type)
{
  init_toml_config();     // process as early as possible, but that depends on SDR software

  const BandAction::Band_Info bi = get_band_info();
  switch (bi)
  {
  case BandAction::Band_Info::info_not_loaded:
    snprintf(band_disp_text, 255, "rtl_sdr_extio.cfg not loaded!");
    break;
  case BandAction::Band_Info::info_parse_error:
    snprintf(band_disp_text, 255, "Error in rtl_sdr_extio.cfg!");
    break;
  case BandAction::Band_Info::info_disabled:
    snprintf(band_disp_text, 255, "rtl_sdr_extio.cfg is not enabled");
    break;
  case BandAction::Band_Info::info_no_bands:
    snprintf(band_disp_text, 255, "No bands defined");
    break;
  case BandAction::Band_Info::info_ok:
    snprintf(band_disp_text, 255, "");
    break;
  }
  update_band_text.store(true);

  strcpy_s(name, 63, "Realtek");
  strcpy_s(model, 15, "RTL2832U-SDR");
  name[63] = 0;
  model[15] = 0;

  if (!SDRsupportsSamplePCMU8)
  {
    extHWtype = exthwUSBdata16; // 16-bit samples
    SDRLOG(extHw_MSG_DEBUG, "InitHW() with sample type PCM16");
  }
  else
  {
    extHWtype = exthwUSBdataU8; // 8bit samples
    SDRLOG(extHw_MSG_DEBUG, "InitHW() with sample type PCMU8");
  }

  type = extHWtype;
  return TRUE;
}

extern "C"
int LIBRTL_API EXTIO_CALL GetStatus()
{
  /* dummy function */
  return 0;
}

extern "C"
bool  LIBRTL_API EXTIO_CALL OpenHW()
{
  SDRLOG(extHw_MSG_DEBUG, "OpenHW()");
  CreateGUI();

  bool r = open_selected_rtl_device();
  if (!r)
  {
    SDRLOG(extHw_MSG_ERROR, "OpenHW(): error opening RTL device");
    if (RtlSelectedDeviceIdx >= MAX_RTL_DEVICES
      || RtlDeviceList[RtlSelectedDeviceIdx].dev_idx >= MAX_RTL_DEVICES)
    {
      gui_show_missing_device(0);  // 0 == OpenHW(), 1 == StartHW()
    }
    return r;
  }
  post_update_gui_init();  // post_update_gui_fields();

  Start_ConnCheck_Thread();

  return true;
}


static CtrlFlagT _setHwLO_check_bands(int64_t freq)
{
  static std::string last_band_name{};
  static char acMsg[256];
  static bool last_was_undefined_band = false;
  CtrlFlagT changed_flags = 0;
  const BandAction::Band_Info bi = get_band_info();
  if (bi != BandAction::Band_Info::info_ok)
    return changed_flags;

  if (nxt.LO_freq.load() == freq && !last_band_name.empty())
    return changed_flags;

  const BandAction* new_band = update_band_action(double(freq));
  if (!new_band)
  {
    if (!last_was_undefined_band)
    {
      snprintf(band_disp_text, 255, "Band: undefined");
      last_was_undefined_band = true;
      update_band_text.store(true);
      SDRLG(extHw_MSG_LOG, "new band name: %s", band_disp_text);
    }
    return changed_flags;
  }

  // we are now moving into a new band with some defined action(s)
  const BandAction& ba = *new_band;   // have a shorter alias

  const std::string& new_band_name = (ba.name.has_value()) ? ba.name.value() : ba.id;
  const bool update_band_name = is_gui_available() && (last_band_name.empty() || new_band_name != last_band_name);
  if (update_band_name)
  {
    last_band_name = new_band_name;
    last_was_undefined_band = false;
    snprintf(band_disp_text, 255, "Band: %s", new_band_name.c_str());
    update_band_text.store(true);
    SDRLG(extHw_MSG_LOG, "new band name: '%s'", band_disp_text);
  }

  if (ba.sampling_mode)
  {
    if (ba.sampling_mode.value() == 'I')
      nxt.sampling_mode = 1;
    else if (ba.sampling_mode.value() == 'Q')
      nxt.sampling_mode = 2;
    else if (ba.sampling_mode.value() == 'C')
      nxt.sampling_mode = 0;
    changed_flags |= CtrlFlags::sampling_mode;  // direct sampling mode;
  }

  if (ba.rtl_digital_agc)
  {
    nxt.rtl_agc = ba.rtl_digital_agc.value() ? 1 : 0;
    changed_flags |= CtrlFlags::rtl_agc;  // rtl agc
  }

  if (ba.tuner_rf_agc)
  {
    nxt.tuner_rf_agc = ba.tuner_rf_agc.value() ? 1 : 0;
    changed_flags |= CtrlFlags::rf_agc;
  }

  if (ba.tuner_if_agc)
  {
    nxt.tuner_if_agc = ba.tuner_if_agc.value() ? 1 : 0;
    changed_flags |= CtrlFlags::if_agc_gain;
  }

  if (ba.tuning_sideband)
  {
    if (ba.tuning_sideband.value() == 'L')
      nxt.USB_sideband = 0;
    else if (ba.tuning_sideband.value() == 'U')
      nxt.USB_sideband = 1;
    changed_flags |= CtrlFlags::tuner_sideband;
  }

  if (ba.gpio_button0)
    nxt.GPIO[0] = ba.gpio_button0.value() ? 1 : 0;
  if (ba.gpio_button1)
    nxt.GPIO[1] = ba.gpio_button0.value() ? 1 : 0;
  if (ba.gpio_button2)
    nxt.GPIO[2] = ba.gpio_button0.value() ? 1 : 0;
  if (ba.gpio_button3)
    nxt.GPIO[3] = ba.gpio_button0.value() ? 1 : 0;
  if (ba.gpio_button4)
    nxt.GPIO[4] = ba.gpio_button0.value() ? 1 : 0;
  if (ba.gpio_button0 || ba.gpio_button1 || ba.gpio_button2 || ba.gpio_button3 || ba.gpio_button4)
    changed_flags |= CtrlFlags::gpio;

  if (ba.tuner_rf_gain_db)
  {
    nxt.tuner_rf_agc = 0;    // also deactivate AGC
    const int rf_gain_tenth_db = int(ba.tuner_rf_gain_db.value() * 10.0);
    if (GotTunerInfo && tunerNo < tuners::N)
    {
      int gainIdx = nearestGainIdx(rf_gain_tenth_db, tuners::rf_gains[tunerNo].gain, tuners::rf_gains[tunerNo].num);
      if (gainIdx >= 0 && gainIdx < tuners::rf_gains[tunerNo].num)
        nxt.rf_gain = tuners::rf_gains[tunerNo].gain[gainIdx];
    }
    changed_flags |= (CtrlFlags::rf_agc | CtrlFlags::rf_gain);
  }
  else if (ba.tuner_rf_gain_db && GotTunerInfo && tunerNo < tuners::N)
  {
    nxt.tuner_rf_agc = 0;    // also deactivate AGC
    const int rf_gain_tenth_db = int(ba.tuner_rf_gain_db.value() * 10.0);
    int gainIdx = nearestGainIdx(rf_gain_tenth_db, tuners::rf_gains[tunerNo].gain, tuners::rf_gains[tunerNo].num);
    SetAttenuator(gainIdx);
  }

  if (ba.tuner_if_gain_db)
  {
    nxt.tuner_if_agc = 0;    // also deactivate AGC
    int if_gain_tenth_db = int(ba.tuner_if_gain_db.value() * 10.0);
    if (GotTunerInfo && tunerNo < tuners::N)
    {
      nxt.if_gain_idx = nearestGainIdx(if_gain_tenth_db, tuners::if_gains[tunerNo].gain, tuners::if_gains[tunerNo].num);
      if (nxt.if_gain_idx >= 0 && nxt.if_gain_idx < tuners::if_gains[tunerNo].num)
        nxt.if_gain_val = tuners::if_gains[tunerNo].gain[nxt.if_gain_idx];

    }
    changed_flags |= CtrlFlags::if_agc_gain;
  }
  else if (ba.tuner_if_gain_db && GotTunerInfo && tunerNo < tuners::N)
  {
    nxt.tuner_if_agc = 0;    // also deactivate AGC
    const int if_gain_tenth_db = int(ba.tuner_if_gain_db.value() * 10.0);

    int if_gain_idx = nearestGainIdx(if_gain_tenth_db, tuners::if_gains[tunerNo].gain, tuners::if_gains[tunerNo].num);
    ExtIoSetMGC(if_gain_idx);
  }


  //if (!changed_flags.is_empty())  // update GUI fields on changes
  if (changed_flags || update_band_name)
    post_update_gui_fields();

  //if (any(changed_flags))
  //    cmd_everything ! - if there's anything to command
  //    changed_flags.set(CtrlFlags::everything);

  return changed_flags;
}


extern "C"
long LIBRTL_API EXTIO_CALL SetHWLO(long freq)
{
  CtrlFlagT change_flags = _setHwLO_check_bands(freq);
  nxt.LO_freq.store(freq); // +nxt.band_center_LO_delta;
  SDRLOG(extHw_MSG_DEBUG, "SetHWLO() -> trigger_control()");
  trigger_control(change_flags | CtrlFlags::freq);
  return 0;
}


extern "C"
int64_t LIBRTL_API EXTIO_CALL SetHWLO64(int64_t freq)
{
  CtrlFlagT change_flags = _setHwLO_check_bands(freq);
  nxt.LO_freq.store(freq); // +nxt.band_center_LO_delta;
  SDRLOG(extHw_MSG_DEBUG, "SetHWLO64() -> trigger_control()");
  trigger_control(change_flags | CtrlFlags::freq);
  return 0;
}


extern "C"
void LIBRTL_API EXTIO_CALL TuneChanged64(int64_t tunefreq)
{
  nxt.tune_freq = tunefreq;
}

extern "C"
void LIBRTL_API EXTIO_CALL TuneChanged(long tunefreq)
{
  nxt.tune_freq = tunefreq;
}

extern "C"
int64_t LIBRTL_API EXTIO_CALL GetTune64(void)
{
  return nxt.tune_freq;
}

extern "C"
long LIBRTL_API EXTIO_CALL GetTune(void)
{
  return (long)(nxt.tune_freq);
}


extern "C"
int LIBRTL_API EXTIO_CALL StartHW(long freq)
{
  char acMsg[256];
  SDRLG(extHw_MSG_DEBUG, "StartHW() with device handle 0x%p", RtlSdrDev);

  Stop_ConnCheck_Thread();

  while (!RtlSdrDev || !is_device_handle_valid())
  {
    if (!RtlSdrDev)
      SDRLOG(extHw_MSG_ERROR, "StartHW(): fail without open device");
    else
      SDRLOG(extHw_MSG_ERROR, "StartHW(): failed with invalid device handle");

    ThreadStreamToSDR = false;
    Stop_RX_Thread();
    close_rtl_device();
    uint32_t N = retrieve_devices();
    if (N)
    {
      bool ok = open_selected_rtl_device();
      if (ok && N == 1)
      {
        SDRLOG(extHw_MSG_WARNING, "Starting with the only available device");
        post_update_gui_init();  // post_update_gui_fields();
        gui_show();
        break;
      }
    }
    EnableGUIControlsAtStop();
    post_update_gui_init();  // post_update_gui_fields();
    gui_show();
    // gui_show_invalid_device();
    // EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Stop);
    return -1;  // return smallest non-error amount - but Stop!
  }

  if (SDRsupportsSamplePCMU8)
    SDRLOG(extHw_MSG_DEBUG, "StartHW(): PCMU8 is supported");
  else
    SDRLOG(extHw_MSG_DEBUG, "StartHW(): PCMU8 is NOT supported");

  if (exthwUSBdata16 == extHWtype)
    SDRLOG(extHw_MSG_DEBUG, "StartHW(): using sample type PCM16");
  else if (exthwUSBdataU8 == extHWtype)
    SDRLOG(extHw_MSG_DEBUG, "StartHW(): using sample type PCMU8");
  else
    SDRLOG(extHw_MSG_DEBUG, "StartHW(): using 'other' sample type - NOT PCMU8 or PCM16!");

  ThreadStreamToSDR = true;
  if (Start_RX_Thread() < 0)
  {
    SDRLOG(extHw_MSG_ERROR, "StartHW(): Error to start streaming thread");
    return -1;
  }
  SDRLOG(extHw_MSG_DEBUG, "StartHW(): Started streaming thread");

  commandEverything = true;
  SetHWLO(freq);

  DisableGUIControlsAtStart();

  // blockSize is independent of decimation!
  // else, we get just 64 = 512 / 8 I/Q Samples with 1 kB bufferSize!
  int numIQpairs = buffer_len / 2;

  snprintf(acMsg, 255, "StartHW() = %d. Callback will deliver %d I/Q pairs per call", numIQpairs, numIQpairs);
  SDRLOG(extHw_MSG_DEBUG, acMsg);

  return numIQpairs;
}


extern "C"
int64_t LIBRTL_API EXTIO_CALL GetHWLO64()
{
  return nxt.LO_freq;
}

extern "C"
long LIBRTL_API EXTIO_CALL GetHWLO()
{
  return (long)nxt.LO_freq;
}


extern "C"
long LIBRTL_API EXTIO_CALL GetHWSR()
{
  long sr = long(rates::tab[nxt.srate_idx].valueInt);
#if ( FULL_DECIMATION )
  sr /= nxt.decimation;
#endif
  return sr;
}

extern "C"
int LIBRTL_API EXTIO_CALL ExtIoGetSrates(int srate_idx, double* samplerate)
{
  if (srate_idx < rates::N)
  {
#if ( FULL_DECIMATION )
    *samplerate = rates::tab[srate_idx].value / nxt.decimation;
#else
    * samplerate = rates::tab[srate_idx].value;
#endif
    return 0;
  }
  return 1; // ERROR
}

extern "C"
long LIBRTL_API EXTIO_CALL ExtIoGetBandwidth(int srate_idx)
{
  if (srate_idx < rates::N)
  {
    // ~ 3/4 of spectrum usable
    long bw = rates::tab[srate_idx].valueInt * 3L / (nxt.decimation * 4L);
    if (nxt.tuner_bw && nxt.tuner_bw * 1000L < bw)
      bw = nxt.tuner_bw * 1000L;
    return bw;
  }
  return 0; // ERROR
}

extern "C"
int  LIBRTL_API EXTIO_CALL ExtIoGetActualSrateIdx(void)
{
  return nxt.srate_idx;
}

extern "C"
int  LIBRTL_API EXTIO_CALL ExtIoSetSrate(int srate_idx)
{
  if (srate_idx >= 0 && srate_idx < rates::N)
  {
    SDRLOG(extHw_MSG_DEBUG, "ExtIoSetSrate() -> trigger_control()");
    gui_SetSrate(srate_idx);
    nxt.srate_idx = srate_idx;
    trigger_control(CtrlFlags::srate);
    EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_SampleRate);  // Signal application
    return 0;
  }
  return 1; // ERROR
}

extern "C"
int  LIBRTL_API EXTIO_CALL GetAttenuators(int atten_idx, float* attenuation)
{
  if (atten_idx < n_rf_gains)
  {
    *attenuation = rf_gains[atten_idx] / 10.0F;
    return 0;
  }
  return 1; // End or Error
}

extern "C"
int  LIBRTL_API EXTIO_CALL GetActualAttIdx(void)
{
  for (int i = 0; i < n_rf_gains; i++)
    if (nxt.rf_gain == rf_gains[i])
      return i;
  return -1;
}

extern "C"
int  LIBRTL_API EXTIO_CALL SetAttenuator(int atten_idx)
{
  if (atten_idx<0 || atten_idx > n_rf_gains)
    return -1;

  gui_SetAttenuator(atten_idx);

  nxt.rf_gain = rf_gains[atten_idx];
  SDRLOG(extHw_MSG_DEBUG, "SetAttenuator() -> trigger_control()");
  trigger_control(CtrlFlags::rf_gain);
  return 0;
}


extern "C"
int  LIBRTL_API EXTIO_CALL ExtIoGetMGCs(int mgc_idx, float* gain)
{
  if (mgc_idx < n_if_gains)
  {
    *gain = if_gains[mgc_idx] / 10.0F;
    return 0;
  }
  return 1; // End or Error
}

extern "C"
int  LIBRTL_API EXTIO_CALL ExtIoGetActualMgcIdx(void)
{
  for (int i = 0; i < n_if_gains; i++)
    if (nxt.if_gain_val == if_gains[i])
      return i;
  return -1;
}

extern "C"
int  LIBRTL_API EXTIO_CALL ExtIoSetMGC(int mgc_idx)
{
  if (mgc_idx<0 || mgc_idx > n_if_gains)
    return -1;

  gui_SetMGC(mgc_idx);

  nxt.if_gain_val = if_gains[mgc_idx];
  nxt.if_gain_idx = mgc_idx;
  SDRLOG(extHw_MSG_DEBUG, "ExtIoSetMGC() -> trigger_control()");
  trigger_control(CtrlFlags::if_agc_gain);
  return 0;
}


#if WITH_AGCS

extern "C"
int   LIBRTL_API EXTIO_CALL ExtIoGetAGCs(int agc_idx, char* text)
{
  switch (agc_idx)
  {
  case 0:   snprintf(text, 16, "%s", "IF");      return 0;
  case 1:   snprintf(text, 16, "%s", "RF:A IF"); return 0;
  case 2:   snprintf(text, 16, "%s", "AGC");     return 0;
  default:  return -1;  // ERROR
  }
  return -1;  // ERROR
}

extern "C"
int   LIBRTL_API EXTIO_CALL ExtIoGetActualAGCidx(void)
{
  return HDSDR_AGC;
}

extern "C"
int   LIBRTL_API EXTIO_CALL ExtIoSetAGC(int agc_idx)
{
  WPARAM RF_val = BST_UNCHECKED; // BST_CHECKED;
  WPARAM IF_val = BST_UNCHECKED; // BST_CHECKED;
  //WPARAM DIG_val = BST_UNCHECKED; // BST_CHECKED;
  HDSDR_AGC = agc_idx;
  switch (agc_idx)
  {
  case 0:   break; // "IF"  
  case 1:   RF_val = BST_CHECKED;  break; // "RF:A IF"
  default:
  case 2:   RF_val = IF_val = BST_CHECKED;  break;  // "AGC"
  }
  SendMessage(GetDlgItem(h_dialog, IDC_TUNER_RF_AGC), BM_SETCHECK, RF_val, NULL);
  SendMessage(GetDlgItem(h_dialog, IDC_TUNER_IF_AGC), BM_SETCHECK, IF_val, NULL);
  //SendMessage(GetDlgItem(h_dialog, IDC_RTL_DIG_AGC), BM_SETCHECK, DIG_val, NULL);
  return 0;
}

extern "C"
int   LIBRTL_API EXTIO_CALL ExtIoShowMGC(int agc_idx)
{
  // return 1 - shows MGC == IF slider
  switch (agc_idx)
  {
  default:
  case 0:   return 1;  // "IF"  
  case 1:   return 1;  // "RF AGC IF"
  case 2:   return 0;  // "RF+IF AGC"
  }
}

#endif /* WITH_AGCS */

//---------------------------------------------------------------------------

enum class Setting {
  ID = 0
  , SRATE_IDX
  , TUNER_BW
  , TUNER_RF_AGC
  , RTL_DIG_AGC
  , CALIB_PPM
  , MANUAL_RF_GAIN
  , BUFFER_SIZE_IDX
  , E4K_OFFSET_TUNE
  , DIRECT_SAMPLING_MODE
  , TUNER_IF_AGC              // int nxt.tuner_if_agc = 1
  , MANUAL_IF_GAIN_IDX        // int nxt.if_gain_idx = 1
  , MANUAL_IF_GAIN_VAL        // int nxt.if_gain = 1
  , R820T_BAND_CENTER_SEL     // int nxt.band_center_sel = 0
  , R820T_BAND_CENTER_DELTA   // int nxt.band_center_LO_delta = 0
  , R820T_USB_SIDEBAND        // int nxt.USB_sideband = 0

  , RTL_GPIO_A_PIN_EN
  , RTL_GPIO_A_INVERT
  , RTL_GPIO_A_VALUE
  , RTL_GPIO_A_LABEL

  , RTL_GPIO_B_PIN_EN
  , RTL_GPIO_B_INVERT
  , RTL_GPIO_B_VALUE
  , RTL_GPIO_B_LABEL

  , RTL_GPIO_C_PIN_EN
  , RTL_GPIO_C_INVERT
  , RTL_GPIO_C_VALUE
  , RTL_GPIO_C_LABEL

  , RTL_GPIO_D_PIN_EN
  , RTL_GPIO_D_INVERT
  , RTL_GPIO_D_VALUE
  , RTL_GPIO_D_LABEL

  , RTL_GPIO_E_PIN_EN
  , RTL_GPIO_E_INVERT
  , RTL_GPIO_E_VALUE
  , RTL_GPIO_E_LABEL

  , RTL_IMPULSE_NOICE_CANCEL

  , RTL_AAGC_RF_EN
  , RTL_AAGC_RF_INV
  , RTL_AAGC_RF_MIN
  , RTL_AAGC_RF_MAX

  , RTL_AAGC_IF_EN
  , RTL_AAGC_IF_INV
  , RTL_AAGC_IF_MIN
  , RTL_AAGC_IF_MAX

  , RTL_AAGC_LG_LOCK
  , RTL_AAGC_LG_UNLOCK
  , RTL_AAGC_LG_IFR

  , RTL_AAGC_VTOP1
  , RTL_AAGC_VTOP2
  , RTL_AAGC_VTOP3
  , RTL_AAGC_KRF1
  , RTL_AAGC_KRF2
  , RTL_AAGC_KRF3
  , RTL_AAGC_KRF4

  , NUM   // Last One == Amount
};


extern "C"
int   LIBRTL_API EXTIO_CALL ExtIoGetSetting(int idx, char* description, char* value)
{
  int atm_tmp_pin, atm_tmp_inv, atm_tmp_en;
  int atm_tmp_i;

  switch (Setting(idx))
  {
  case Setting::ID:
    snprintf(description, 1024, "%s", "Identifier");
    snprintf(value, 1024, "%s", SETTINGS_IDENTIFIER);
    return 0;
  case Setting::SRATE_IDX:
    atm_tmp_i = nxt.srate_idx;
    snprintf(description, 1024, "%s", "SampleRateIdx");
    snprintf(value, 1024, "%d", atm_tmp_i);
    return 0;
  case Setting::TUNER_BW:
    snprintf(description, 1024, "%s", "TunerBandwidth in kHz (only few tuner models) - 0 for automatic");
    snprintf(value, 1024, "%d", nxt.tuner_bw.load());
    return 0;
  case Setting::TUNER_RF_AGC:
    snprintf(description, 1024, "%s", "Tuner_RF_AGC");
    snprintf(value, 1024, "%d", nxt.tuner_rf_agc.load());
    return 0;
  case Setting::RTL_DIG_AGC:
    snprintf(description, 1024, "%s", "RTL_AGC");
    snprintf(value, 1024, "%d", nxt.rtl_agc.load());
    return 0;
  case Setting::CALIB_PPM:
    snprintf(description, 1024, "%s", "Frequency_Correction");
    snprintf(value, 1024, "%d", nxt.freq_corr_ppm.load());
    return 0;
  case Setting::MANUAL_RF_GAIN:
    snprintf(description, 1024, "%s", "Tuner_RF_Gain");
    snprintf(value, 1024, "%d", nxt.rf_gain.load());
    return 0;
  case Setting::BUFFER_SIZE_IDX:
    snprintf(description, 1024, "%s", "Buffer_Size");
    snprintf(value, 1024, "%d", bufferSizeIdx.load());
    return 0;
  case Setting::E4K_OFFSET_TUNE:
    snprintf(description, 1024, "%s", "Tuner-E4000_Offset_Tuning");
    snprintf(value, 1024, "%d", nxt.offset_tuning.load());
    return 0;
  case Setting::DIRECT_SAMPLING_MODE:
    snprintf(description, 1024, "%s", "Sampling_Mode");
    snprintf(value, 1024, "%d", nxt.sampling_mode.load());
    return 0;

  case Setting::TUNER_IF_AGC:             // int nxt.tuner_if_agc = 1
    snprintf(description, 1024, "%s", "R820T/2_IF_AGC");
    snprintf(value, 1024, "%d", nxt.tuner_if_agc.load());
    return 0;
  case Setting::MANUAL_IF_GAIN_IDX:       // int nxt.if_gain_idx = 1
    snprintf(description, 1024, "%s", "R820T/2_manual_IF_GAIN_idx");
    snprintf(value, 1024, "%d", nxt.if_gain_idx.load());
    return 0;
  case Setting::MANUAL_IF_GAIN_VAL:       // int nxt.if_gain = 1
    snprintf(description, 1024, "%s", "R820T/2_manual_IF_GAIN_value");
    snprintf(value, 1024, "%d", nxt.if_gain_val.load());
    return 0;
  case Setting::R820T_BAND_CENTER_SEL:    // int nxt.band_center_sel = 0
    snprintf(description, 1024, "%s", "R820T/2_Band_Center_Selection");
    snprintf(value, 1024, "%d", nxt.band_center_sel.load());
    return 0;
  case Setting::R820T_BAND_CENTER_DELTA:  // int nxt.band_center_LO_delta = 0
    snprintf(description, 1024, "%s", "R820T/2_Band_Center_in_Hz");
    snprintf(value, 1024, "%d", nxt.band_center_LO_delta.load());
    return 0;
  case Setting::R820T_USB_SIDEBAND:       // int nxt.USB_sideband = 0
    snprintf(description, 1024, "%s", "R820T/2_Tune_Upper_Sideband");
    snprintf(value, 1024, "%d", nxt.USB_sideband.load());
    return 0;

  case Setting::RTL_GPIO_A_PIN_EN:
    atm_tmp_en = GPIO_en[0].load();
    atm_tmp_pin = GPIO_pin[0].load();
    snprintf(description, 1024, "%s", "GPIO Button A: Pin Number (0 .. 7), -1 for BiasT, other values to disable");
    snprintf(value, 1024, "%d", atm_tmp_en ? atm_tmp_pin : (-10 - atm_tmp_pin));
    return 0;
  case Setting::RTL_GPIO_A_INVERT:
    atm_tmp_inv = GPIO_inv[0].load();
    snprintf(description, 1024, "%s", "GPIO Button A: Invert Pin Value?: 0 = value as CheckBox. 1 = inverted");
    snprintf(value, 1024, "%d", atm_tmp_inv);
    return 0;
  case Setting::RTL_GPIO_A_VALUE:
    snprintf(description, 1024, "%s", "GPIO Button A: Checkbox Value?: 0 = unchecked. 1 = checked");
    snprintf(value, 1024, "%d", nxt.GPIO[0].load());
    return 0;
  case Setting::RTL_GPIO_A_LABEL:
    snprintf(description, 1024, "%s", "GPIO Button A: Label to be shown at CheckBox - max size 16");
    snprintf(value, 1024, "%s", &GPIO_txt[0][0]);
    return 0;
  case Setting::RTL_GPIO_B_PIN_EN:
    atm_tmp_en = GPIO_en[1].load();
    atm_tmp_pin = GPIO_pin[1].load();
    snprintf(description, 1024, "%s", "GPIO Button B: Pin Number (0 .. 7), -1 for BiasT, other values to disable");
    snprintf(value, 1024, "%d", atm_tmp_en ? atm_tmp_pin : (-10 - atm_tmp_pin));
    return 0;
  case Setting::RTL_GPIO_B_INVERT:
    atm_tmp_inv = GPIO_inv[1].load();
    snprintf(description, 1024, "%s", "GPIO Button B: Invert Pin Value?: 0 = value as CheckBox. 1 = inverted");
    snprintf(value, 1024, "%d", atm_tmp_inv);
    return 0;
  case Setting::RTL_GPIO_B_VALUE:
    snprintf(description, 1024, "%s", "GPIO Button B: Checkbox Value?: 0 = unchecked. 1 = checked");
    snprintf(value, 1024, "%d", nxt.GPIO[1].load());
    return 0;
  case Setting::RTL_GPIO_B_LABEL:
    snprintf(description, 1024, "%s", "GPIO Button B: Label to be shown at CheckBox - max size 16");
    snprintf(value, 1024, "%s", &GPIO_txt[1][0]);
    return 0;
  case Setting::RTL_GPIO_C_PIN_EN:
    atm_tmp_en = GPIO_en[2].load();
    atm_tmp_pin = GPIO_pin[2].load();
    snprintf(description, 1024, "%s", "GPIO Button C: Pin Number (0 .. 7), -1 for BiasT, other values to disable");
    snprintf(value, 1024, "%d", atm_tmp_en ? atm_tmp_pin : (-10 - atm_tmp_pin));
    return 0;
  case Setting::RTL_GPIO_C_INVERT:
    atm_tmp_inv = GPIO_inv[2].load();
    snprintf(description, 1024, "%s", "GPIO Button C: Invert Pin Value?: 0 = value as CheckBox. 1 = inverted");
    snprintf(value, 1024, "%d", atm_tmp_inv);
    return 0;
  case Setting::RTL_GPIO_C_VALUE:
    snprintf(description, 1024, "%s", "GPIO Button C: Checkbox Value?: 0 = unchecked. 1 = checked");
    snprintf(value, 1024, "%d", nxt.GPIO[2].load());
    return 0;
  case Setting::RTL_GPIO_C_LABEL:
    snprintf(description, 1024, "%s", "GPIO Button C: Label to be shown at CheckBox - max size 16");
    snprintf(value, 1024, "%s", &GPIO_txt[2][0]);
    return 0;
  case Setting::RTL_GPIO_D_PIN_EN:
    atm_tmp_en = GPIO_en[3].load();
    atm_tmp_pin = GPIO_pin[3].load();
    snprintf(description, 1024, "%s", "GPIO Button D: Pin Number (0 .. 7), -1 for BiasT, other values to disable");
    snprintf(value, 1024, "%d", atm_tmp_en ? atm_tmp_pin : (-10 - atm_tmp_pin));
    return 0;
  case Setting::RTL_GPIO_D_INVERT:
    atm_tmp_inv = GPIO_inv[3].load();
    snprintf(description, 1024, "%s", "GPIO Button D: Invert Pin Value?: 0 = value as CheckBox. 1 = inverted");
    snprintf(value, 1024, "%d", atm_tmp_inv);
    return 0;
  case Setting::RTL_GPIO_D_VALUE:
    snprintf(description, 1024, "%s", "GPIO Button D: Checkbox Value?: 0 = unchecked. 1 = checked");
    snprintf(value, 1024, "%d", nxt.GPIO[3].load());
    return 0;
  case Setting::RTL_GPIO_D_LABEL:
    snprintf(description, 1024, "%s", "GPIO Button D: Label to be shown at CheckBox - max size 16");
    snprintf(value, 1024, "%s", &GPIO_txt[3][0]);
    return 0;
  case Setting::RTL_GPIO_E_PIN_EN:
    atm_tmp_en = GPIO_en[4].load();
    atm_tmp_pin = GPIO_pin[4].load();
    snprintf(description, 1024, "%s", "GPIO Button E: Pin Number (0 .. 7), -1 for BiasT, other values to disable");
    snprintf(value, 1024, "%d", atm_tmp_en ? atm_tmp_pin : (-10 - atm_tmp_pin));
    return 0;
  case Setting::RTL_GPIO_E_INVERT:
    atm_tmp_inv = GPIO_inv[4].load();
    snprintf(description, 1024, "%s", "GPIO Button E: Invert Pin Value?: 0 = value as CheckBox. 1 = inverted");
    snprintf(value, 1024, "%d", atm_tmp_inv);
    return 0;
  case Setting::RTL_GPIO_E_VALUE:
    snprintf(description, 1024, "%s", "GPIO Button E: Checkbox Value?: 0 = unchecked. 1 = checked");
    snprintf(value, 1024, "%d", nxt.GPIO[4].load());
    return 0;
  case Setting::RTL_GPIO_E_LABEL:
    snprintf(description, 1024, "%s", "GPIO Button E: Label to be shown at CheckBox - max size 16");
    snprintf(value, 1024, "%s", &GPIO_txt[4][0]);
    return 0;

  case Setting::RTL_IMPULSE_NOICE_CANCEL:
    snprintf(description, 1024, "%s", "Impulse Noise Cancellation in RTL2832U. -1: don't touch, 0: off, 1: on");
    snprintf(value, 1024, "%d", nxt.rtl_impulse_noise_cancellation.load());
    return 0;

  case Setting::RTL_AAGC_RF_EN:
    snprintf(description, 1024, "%s", "Enable RF-AAGC - feedback to tuner. -1: don't touch or 0 .. 1");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_rf_en.load());
    return 0;
  case Setting::RTL_AAGC_RF_INV:
    snprintf(description, 1024, "%s", "Invert RF-AAGC output - feedback to tuner. -1: don't touch or 0 .. 1");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_rf_inv.load());
    return 0;
  case Setting::RTL_AAGC_RF_MIN:
    snprintf(description, 1024, "%s", "Min Gain for RF-AAGC - feedback to tuner. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_rf_min.load());
    return 0;
  case Setting::RTL_AAGC_RF_MAX:
    snprintf(description, 1024, "%s", "Max Gain for RF-AAGC - feedback to tuner. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_rf_max.load());
    return 0;

  case Setting::RTL_AAGC_IF_EN:
    snprintf(description, 1024, "%s", "Enable IF-AAGC - feedback to tuner. -1: don't touch or 0 .. 1");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_if_en.load());
    return 0;
  case Setting::RTL_AAGC_IF_INV:
    snprintf(description, 1024, "%s", "Invert IF-AAGC output - feedback to tuner. -1: don't touch or 0 .. 1");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_if_inv.load());
    return 0;
  case Setting::RTL_AAGC_IF_MIN:
    snprintf(description, 1024, "%s", "Min Gain for IF-AGC - feedback to tuner. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_if_min.load());
    return 0;
  case Setting::RTL_AAGC_IF_MAX:
    snprintf(description, 1024, "%s", "Max Gain for IF-AGC - feedback to tuner. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_if_max.load());
    return 0;

  case Setting::RTL_AAGC_LG_LOCK:
    snprintf(description, 1024, "%s", "AAGC Loop Gain Lock. -1: don't touch or 0 .. 31");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_lg_lock.load());
    return 0;
  case Setting::RTL_AAGC_LG_UNLOCK:
    snprintf(description, 1024, "%s", "AAGC Loop Gain UnLock. -1: don't touch or 0 .. 31");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_lg_unlock.load());
    return 0;
  case Setting::RTL_AAGC_LG_IFR:
    snprintf(description, 1024, "%s", "AAGC Loop Gain Interference. -1: don't touch or 0 .. 31");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_lg_ifr .load());
    return 0;

  case Setting::RTL_AAGC_VTOP1:
    snprintf(description, 1024, "%s", "RTL AGC gain distribution RF / IF: Take-Over-Point1 aka VTOP1. -1: don't touch or 0 .. 63");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_vtop[0].load());
    return 0;
  case Setting::RTL_AAGC_VTOP2:
    snprintf(description, 1024, "%s", "RTL AGC gain distribution RF / IF: Take-Over-Point2 aka VTOP2. -1: don't touch or 0 .. 63");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_vtop[1].load());
    return 0;
  case Setting::RTL_AAGC_VTOP3:
    snprintf(description, 1024, "%s", "RTL AGC gain distribution RF / IF: Take-Over-Point3 aka VTOP3. -1: don't touch or 0 .. 63");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_vtop[2].load());
    return 0;
  case Setting::RTL_AAGC_KRF1:
    snprintf(description, 1024, "%s", "RTL AGC RF gain degrade ratio 1 aka KRF1. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_krf[0].load());
    return 0;
  case Setting::RTL_AAGC_KRF2:
    snprintf(description, 1024, "%s", "RTL AGC RF gain degrade ratio 2 aka KRF2. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_krf[1].load());
    return 0;
  case Setting::RTL_AAGC_KRF3:
    snprintf(description, 1024, "%s", "RTL AGC RF gain degrade ratio 3 aka KRF3. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_krf[2].load());
    return 0;
  case Setting::RTL_AAGC_KRF4:
    snprintf(description, 1024, "%s", "RTL AGC RF gain degrade ratio 4 aka KRF4. -1: don't touch or 0 .. 255");
    snprintf(value, 1024, "%d", nxt.rtl_aagc_krf[3].load());
    return 0;

  default:
    return -1;  // ERROR
  }
  return -1;  // ERROR
}

extern "C"
void  LIBRTL_API EXTIO_CALL ExtIoSetSetting(int idx, const char* value)
{
  static bool bCompatibleSettings = true;  // initial assumption
  int tempInt;

  init_toml_config();     // process as early as possible, but that depends on SDR software

  // in case settings are not compatible: keep internal defaults
  if (!bCompatibleSettings)
    return;

  switch (Setting(idx))
  {
  case Setting::ID:
    bCompatibleSettings = (0 == strcmp(SETTINGS_IDENTIFIER, value));
    return;
  case Setting::SRATE_IDX:
    tempInt = atoi(value);
    if (tempInt >= 0 && tempInt < rates::N)
      nxt.srate_idx = tempInt;
    return;
  case Setting::TUNER_BW:
    nxt.tuner_bw = atoi(value);
    return;
  case Setting::TUNER_RF_AGC:
    nxt.tuner_rf_agc = atoi(value) ? 1 : 0;
    return;
  case Setting::RTL_DIG_AGC:
    nxt.rtl_agc = atoi(value) ? 1 : 0;
    return;
  case Setting::CALIB_PPM:
    tempInt = atoi(value);
    if (tempInt > MIN_PPM && tempInt < MAX_PPM)
      nxt.freq_corr_ppm = tempInt;
    return;
  case Setting::MANUAL_RF_GAIN:
    nxt.rf_gain = atoi(value);
    return;
  case Setting::BUFFER_SIZE_IDX:
    tempInt = atoi(value);
    if (tempInt >= 0 && tempInt < (sizeof(buffer_sizes) / sizeof(buffer_sizes[0])))
      bufferSizeIdx = tempInt;
    return;
  case Setting::E4K_OFFSET_TUNE:
    nxt.offset_tuning = atoi(value) ? 1 : 0;
    return;
  case Setting::DIRECT_SAMPLING_MODE:
    tempInt = atoi(value);
    if (tempInt < 0)  tempInt = 0;  else if (tempInt > 2) tempInt = 2;
    nxt.sampling_mode = tempInt;
    break;

  case Setting::TUNER_IF_AGC:             // int nxt.tuner_if_agc = 1
    nxt.tuner_if_agc = atoi(value) ? 1 : 0;
    break;
  case Setting::MANUAL_IF_GAIN_IDX:       // int nxt.if_gain_idx = 1
    nxt.if_gain_idx = atoi(value);
    break;
  case Setting::MANUAL_IF_GAIN_VAL:       // int nxt.if_gain = 1
    nxt.if_gain_val = atoi(value);
    break;
  case Setting::R820T_BAND_CENTER_SEL:    // int nxt.band_center_sel = 0
    nxt.band_center_sel = atoi(value);
    break;
  case Setting::R820T_BAND_CENTER_DELTA:  // int nxt.band_center_LO_delta = 0
    nxt.band_center_LO_delta = atoi(value);
    break;
  case Setting::R820T_USB_SIDEBAND:       // int nxt.USB_sideband = 0
    nxt.USB_sideband = atoi(value) ? 1 : 0;
    break;

  case Setting::RTL_GPIO_A_PIN_EN:
    tempInt = atoi(value);
    GPIO_pin[0] = (tempInt >= -1) ? tempInt : (-10 - tempInt);
    GPIO_en[0] = (-1 <= GPIO_pin[0] && GPIO_pin[0] < 8) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_A_INVERT:
    GPIO_inv[0] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_A_VALUE:
    nxt.GPIO[0] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_A_LABEL:
    snprintf(&GPIO_txt[0][0], 15, "%s", value); GPIO_txt[0][15] = 0;
    break;

  case Setting::RTL_GPIO_B_PIN_EN:
    tempInt = atoi(value);
    GPIO_pin[1] = (tempInt >= -1) ? tempInt : (-10 - tempInt);
    GPIO_en[1] = (-1 <= GPIO_pin[1] && GPIO_pin[1] < 8) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_B_INVERT:
    GPIO_inv[1] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_B_VALUE:
    nxt.GPIO[1] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_B_LABEL:
    snprintf(&GPIO_txt[1][0], 15, "%s", value); GPIO_txt[1][15] = 0;
    break;

  case Setting::RTL_GPIO_C_PIN_EN:
    tempInt = atoi(value);
    GPIO_pin[2] = (tempInt >= -1) ? tempInt : (-10 - tempInt);
    GPIO_en[2] = (-1 <= GPIO_pin[2] && GPIO_pin[2] < 8) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_C_INVERT:
    GPIO_inv[2] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_C_VALUE:
    nxt.GPIO[2] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_C_LABEL:
    snprintf(&GPIO_txt[2][0], 15, "%s", value); GPIO_txt[2][15] = 0;
    break;

  case Setting::RTL_GPIO_D_PIN_EN:
    tempInt = atoi(value);
    GPIO_pin[3] = (tempInt >= -1) ? tempInt : (-10 - tempInt);
    GPIO_en[3] = (-1 <= GPIO_pin[3] && GPIO_pin[3] < 8) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_D_INVERT:
    GPIO_inv[3] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_D_VALUE:
    nxt.GPIO[3] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_D_LABEL:
    snprintf(&GPIO_txt[3][0], 15, "%s", value); GPIO_txt[3][15] = 0;
    break;

  case Setting::RTL_GPIO_E_PIN_EN:
    tempInt = atoi(value);
    GPIO_pin[4] = (tempInt >= -1) ? tempInt : (-10 - tempInt);
    GPIO_en[4] = (-1 <= GPIO_pin[4] && GPIO_pin[4] < 8) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_E_INVERT:
    GPIO_inv[4] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_E_VALUE:
    nxt.GPIO[4] = atoi(value) ? 1 : 0;
    break;
  case Setting::RTL_GPIO_E_LABEL:
    snprintf(&GPIO_txt[4][0], 15, "%s", value); GPIO_txt[4][15] = 0;
    break;

  case Setting::RTL_IMPULSE_NOICE_CANCEL:
    nxt.rtl_impulse_noise_cancellation = atoi(value);
    break;

  case Setting::RTL_AAGC_RF_EN:
    nxt.rtl_aagc_rf_en = atoi(value);
    break;
  case Setting::RTL_AAGC_RF_INV:
    nxt.rtl_aagc_rf_inv = atoi(value);
    break;
  case Setting::RTL_AAGC_RF_MIN:
    nxt.rtl_aagc_rf_min = atoi(value);
    break;
  case Setting::RTL_AAGC_RF_MAX:
    nxt.rtl_aagc_rf_max = atoi(value);
    break;

  case Setting::RTL_AAGC_IF_EN:
    nxt.rtl_aagc_if_en = atoi(value);
    break;
  case Setting::RTL_AAGC_IF_INV:
    nxt.rtl_aagc_if_inv = atoi(value);
    break;
  case Setting::RTL_AAGC_IF_MIN:
    nxt.rtl_aagc_if_min = atoi(value);
    break;
  case Setting::RTL_AAGC_IF_MAX:
    nxt.rtl_aagc_if_max = atoi(value);
    break;

  case Setting::RTL_AAGC_LG_LOCK:
    nxt.rtl_aagc_lg_lock = atoi(value);
    break;
  case Setting::RTL_AAGC_LG_UNLOCK:
    nxt.rtl_aagc_lg_unlock = atoi(value);
    break;
  case Setting::RTL_AAGC_LG_IFR:
    nxt.rtl_aagc_lg_ifr = atoi(value);
    break;

  case Setting::RTL_AAGC_VTOP1:
    nxt.rtl_aagc_vtop[0] = atoi(value);
    break;
  case Setting::RTL_AAGC_VTOP2:
    nxt.rtl_aagc_vtop[1] = atoi(value);
    break;
  case Setting::RTL_AAGC_VTOP3:
    nxt.rtl_aagc_vtop[2] = atoi(value);
    break;
  case Setting::RTL_AAGC_KRF1:
    nxt.rtl_aagc_krf[0] = atoi(value);
    break;
  case Setting::RTL_AAGC_KRF2:
    nxt.rtl_aagc_krf[1] = atoi(value);
    break;
  case Setting::RTL_AAGC_KRF3:
    nxt.rtl_aagc_krf[2] = atoi(value);
    break;
  case Setting::RTL_AAGC_KRF4:
    nxt.rtl_aagc_krf[3] = atoi(value);
    break;
  }
}


extern "C"
void LIBRTL_API EXTIO_CALL StopHW()
{
  SDRLOG(extHw_MSG_DEBUG, "StopHW()");
  ThreadStreamToSDR = false;
  Stop_RX_Thread();
  EnableGUIControlsAtStop();
  Start_ConnCheck_Thread();
}

extern "C"
void LIBRTL_API EXTIO_CALL CloseHW()
{
  SDRLOG(extHw_MSG_DEBUG, "CloseHW()");
  ThreadStreamToSDR = false;
  Stop_RX_Thread();
  close_rtl_device();
  DestroyGUI();
}


extern "C"
void LIBRTL_API EXTIO_CALL SetCallback(pfnExtIOCallback funcptr)
{
  gpfnExtIOCallbackPtr = funcptr;
}

extern "C"
void LIBRTL_API  EXTIO_CALL VersionInfo(const char* progname, int ver_major, int ver_minor)
{
  if (!strcmp(progname, "HDSDR")
    && (ver_major >= 3 || (ver_major == 2 && ver_minor > 70)))
  {
    SDRsupportsSamplePCMU8 = true;
    SDRLOG(extHw_MSG_DEBUG, "detected HDSDR > 2.70. => enabling PCMU8");
  }
}

extern "C"
void LIBRTL_API  EXTIO_CALL ExtIoSDRInfo(int extSDRInfo, int additionalValue, void* additionalPtr)
{
  init_toml_config();     // process as early as possible, but that depends on SDR software

  if (extSDRInfo == extSDR_supports_PCMU8)
  {
    SDRsupportsSamplePCMU8 = true;
    SDRLOG(extHw_MSG_DEBUG, "detected SDR with PCMU8 capability => enabling PCMU8");
  }
  else if (extSDRInfo == extSDR_supports_Logging)
    SDRsupportsLogging = true;
  else if (extSDRInfo == extSDR_supports_SampleFormats)
    SDRsupportsSampleFormats = true;
}

struct CallbackContext
{
  void reset()
  {
    receiveBufferIdx = 0;
    printCallbackLen = true;
    acMsg[0] = 0;
  }

  char acMsg[256];
  int receiveBufferIdx;
  bool printCallbackLen;
};

static CallbackContext cb_ctx;


int Start_RX_Thread()
{
  //If already running, exit
  if (RX_thread_handle != INVALID_HANDLE_VALUE)
  {
    SDRLOG(extHw_MSG_ERROR, "Start_RX_Thread(): Error thread still running!");
    return 0;   // all fine
  }

  terminate_RX_Thread = false;

  if (!rcvBufsAllocated)
  {
    for (int k = 0; k <= NUM_BUFFERS_BEFORE_CALLBACK; ++k)
    {
      pcm16_buf[k] = new (std::nothrow) int16_t[MAX_BUFFER_LEN + 1024];
      if (pcm16_buf[k] == 0)
      {
        MessageBox(NULL, TEXT("Couldn't Allocate Sample Buffer!"), TEXT("Error!"), MB_OK | MB_ICONERROR);
        return -1;
      }
    }
    for (int k = 0; k <= NUM_BUFFERS_BEFORE_CALLBACK; ++k)
    {
      rcvBuf[k] = new (std::nothrow) uint8_t[MAX_BUFFER_LEN + 1024];
      if (rcvBuf[k] == 0)
      {
        MessageBox(NULL, TEXT("Couldn't Allocate Sample Buffers!"), TEXT("Error!"), MB_OK | MB_ICONERROR);
        return -1;
      }
    }
    rcvBufsAllocated = true;
  }

  // Reset endpoint
  if (rtlsdr_reset_buffer(RtlSdrDev) < 0)
  {
    SDRLOG(extHw_MSG_ERROR, "Start_RX_Thread(): Error at rtlsdr_reset_buffer()");
    return -1;
  }

  cb_ctx.reset();

  SDRLOG(extHw_MSG_DEBUG, "Starting ASYNC receive thread ..");
  RX_thread_handle = (HANDLE)_beginthread(RX_ThreadProc, 0, NULL);
  if (RX_thread_handle == INVALID_HANDLE_VALUE)
  {
    SDRLOG(extHw_MSG_ERROR, "Start_RX_Thread(): Error at _beginthread()");
    return -1;  // ERROR
  }

  //SetThreadPriority(RX_thread_handle, THREAD_PRIORITY_TIME_CRITICAL);
  return 0;
}

static void RtlSdrCallback(unsigned char* buf, uint32_t len, void* ctx)
{
  if (!buf || !ctx || !gpfnExtIOCallbackPtr || terminate_RX_Thread.load() || len != buffer_len.load())
    return;
  CallbackContext& c = *((CallbackContext*)ctx);

  const int n_samples_per_block = len / 2;

  if (extHWtype == exthwUSBdata16)
  {
    int16_t* short_ptr = pcm16_buf[c.receiveBufferIdx];
    const unsigned char* char_ptr = buf;
    ++c.receiveBufferIdx;
    if (c.receiveBufferIdx >= NUM_BUFFERS_BEFORE_CALLBACK + 1)
      c.receiveBufferIdx = 0;
    for (uint32_t i = 0; i < len; i++)
      short_ptr[i] = int16_t(char_ptr[i]) - int16_t(128);
    if (c.printCallbackLen)
    {
      c.printCallbackLen = false;
      snprintf(c.acMsg, 255, "Callback() with %d raw 16 bit I/Q pairs", n_samples_per_block);
      SDRLOG(extHw_MSG_DEBUG, c.acMsg);
    }
    gpfnExtIOCallbackPtr(n_samples_per_block, 0, 0, short_ptr);
  }
  else // if (extHWtype == exthwUSBdataU8)
  {
    uint8_t* pcm8_buf = rcvBuf[c.receiveBufferIdx];
    const unsigned char* char_ptr = buf;
    ++c.receiveBufferIdx;
    if (c.receiveBufferIdx >= NUM_BUFFERS_BEFORE_CALLBACK + 1)
      c.receiveBufferIdx = 0;
    memcpy(pcm8_buf, buf, len);
    if (c.printCallbackLen)
    {
      c.printCallbackLen = false;
      snprintf(c.acMsg, 255, "Callback() with %d raw 8 Bit I/Q pairs", n_samples_per_block);
      SDRLOG(extHw_MSG_DEBUG, c.acMsg);
    }
    gpfnExtIOCallbackPtr(n_samples_per_block, 0, 0, pcm8_buf);
  }
}

int Stop_RX_Thread()
{
  terminate_RX_Thread = true;
  SDRLOG(extHw_MSG_DEBUG, "Stopping ASYNC receive thread with rtlsdr_cancel_async() ..");
  rtlsdr_cancel_async(RtlSdrDev);
  if (RX_thread_handle == INVALID_HANDLE_VALUE)
    return 0;
  WaitForSingleObject(RX_thread_handle, INFINITE);
  SDRLOG(extHw_MSG_DEBUG, "Stop_RX_Thread(): thread() stopped successfully");
  RX_thread_handle = INVALID_HANDLE_VALUE;
  return 0;
}


void RX_ThreadProc(void* p)
{
  char acMsg[256];
  SDRLG(extHw_MSG_DEBUG, "RX_ThreadProc() with device handle 0x%p", RtlSdrDev);
  // Blocks until rtlsdr_cancel_async() is called
  int r = rtlsdr_read_async(
    RtlSdrDev,
    (rtlsdr_read_async_cb_t)&RtlSdrCallback,
    &cb_ctx,
    0,
    buffer_len.load()
  );

  RX_thread_handle = INVALID_HANDLE_VALUE;

  if (terminate_RX_Thread.load())
    SDRLOG(extHw_MSG_DEBUG, "RX_ThreadProc(): rtlsdr_read_async() finished. Finishing thread.");
  else
  {
    SDRLG(extHw_MSG_WARNING, "RX_ThreadProc(): rtlsdr_read_async() finished unexpected - with %d", r);
    EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Stop);
    close_rtl_device();
  }

  terminate_RX_Thread = true;
  SDRLOG(extHw_MSG_DEBUG, "Stopping ASYNC receive thread with rtlsdr_cancel_async() ..");

  _endthread();
}


int Start_ConnCheck_Thread()
{
  //If already running, exit
  if (ConnCheck_thread_handle != INVALID_HANDLE_VALUE)
  {
    SDRLOG(extHw_MSG_ERROR, "Start_ConnCheck_Thread(): Error thread still running!");
    return 0;   // all fine
  }

  terminate_ConnCheck_Thread = false;

  SDRLOG(extHw_MSG_DEBUG, "Starting ConnCheck thread ..");
  ConnCheck_thread_handle = (HANDLE)_beginthread(ConnCheck_ThreadProc, 0, NULL);
  if (ConnCheck_thread_handle == INVALID_HANDLE_VALUE)
  {
    SDRLOG(extHw_MSG_ERROR, "Start_ConnCheck_Thread(): Error at _beginthread()");
    return -1;  // ERROR
  }

  return 0;
}


int Stop_ConnCheck_Thread()
{
  terminate_ConnCheck_Thread = true;
  SDRLOG(extHw_MSG_DEBUG, "Stopping ConnCheck thread  ..");
  if (ConnCheck_thread_handle == INVALID_HANDLE_VALUE)
    return 0;
  WaitForSingleObject(ConnCheck_thread_handle, INFINITE);
  SDRLOG(extHw_MSG_DEBUG, "Stop_ConnCheck_Thread(): thread() stopped successfully");
  ConnCheck_thread_handle = INVALID_HANDLE_VALUE;
  return 0;
}

void ConnCheck_ThreadProc(void* param)
{
  char acMsg[256];
  SDRLG(extHw_MSG_DEBUG, "ConnCheck_ThreadProc() with device handle 0x%p", RtlSdrDev);
  int counter = 0;

  while (RtlSdrDev && !terminate_ConnCheck_Thread.load())
  {
    Sleep(100);
    if (terminate_ConnCheck_Thread.load())
      break;
    if (++counter <= 5)
      continue;

    counter = 0;
    if (!is_device_handle_valid())
    {
      SDRLOG(extHw_MSG_ERROR, "ConnCheck_ThreadProc(): device handle got invalid!");
      close_rtl_device();
    }
  }

  ConnCheck_thread_handle = INVALID_HANDLE_VALUE;
  SDRLOG(extHw_MSG_DEBUG, "ConnCheck_ThreadProc() finished. Finishing thread.");
  _endthread();
}

