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
#include "resource.h"

#include "LC_ExtIO_Types.h"

#ifdef HAS_WIN_GUI_DLG
#include <Windows.h>
#include <WindowsX.h>
#include <commctrl.h>
#include <process.h>
#include <tchar.h>
#endif

#include "gui_dlg.h"

#define LIBRTL_EXPORTS 1
#include "ExtIO_RTL.h"

#include <stdint.h>
#include <stdio.h>
#include <atomic>
#include <new>


extern int VAR_ALWAYS_PCMU8;
extern int VAR_ALWAYS_PCM16;
#define MAX_DECIMATIONS ( VAR_ALWAYS_PCMU8 ? 1 : 8 )


#ifdef _MSC_VER
#pragma warning(disable : 4996)
#define snprintf  _snprintf
#endif


static const int buffer_sizes[] = { //in kBytes
  1, 2, 4, 8, 16, 32, 64, 128, 256
};


static const char* s_modes[] = {
  "I/Q - sampling of tuner output",
  "pin I: aliases 0 - 14.4 - 28.8 MHz!",
  "pin Q: aliases 0 - 14.4 - 28.8 MHz! (V3)"
};

static constexpr unsigned NUM_GPIO_BUTTONS = ControlVars::NUM_GPIO_BUTTONS;


extern std::atomic_int bufferSizeIdx;
extern std::atomic_int buffer_len;

extern bool SDRsupportsLogging;
extern bool SDRsupportsSamplePCMU8;
extern bool SDRsupportsSampleFormats;
extern std::atomic_bool ThreadStreamToSDR;
extern std::atomic_uint32_t tunerNo;
extern extHWtypeT extHWtype;
extern pfnExtIOCallback gpfnExtIOCallbackPtr;  /* ExtIO Callback */

extern std::atomic_int64_t retune_value;
extern std::atomic_int retune_counter;
extern std::atomic_bool retune_freq;

int nearestBwIdx(int bw);
int nearestGainIdx(int gain, const int* gains, const int n_gains);

static int maxDecimation = 0;


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


#ifdef HAS_WIN_GUI_DLG
static INT_PTR CALLBACK MainDlgProc(HWND, UINT, WPARAM, LPARAM);
static HWND h_dlg = NULL;
#endif

char band_disp_text[255] = { 0 };
std::atomic_bool update_band_text = false;


static inline bool isR82XX()
{
  const int t = tunerNo;
  return (RTLSDR_TUNER_R820T == t || RTLSDR_TUNER_R828D == t || RTLSDR_TUNER_BLOG_V4 == t || RTLSDR_TUNER_BLOG_V4L == t);
}


#ifdef HAS_WIN_GUI_DLG

void CreateGUI()
{
  h_dlg = CreateDialog(hInst, MAKEINTRESOURCE(IDD_RTL_SETTINGS), NULL, (DLGPROC)MainDlgProc);
  if (h_dlg)
  {
    char acMsg[256];
    RECT rect;
    ShowWindow(h_dlg, SW_HIDE);
    if (GetWindowRect(h_dlg, &rect))
    {
      int width = rect.right - rect.left;
      int height = rect.bottom - rect.top;
      SDRLG(extHw_MSG_DEBUG, "Created Window: size %d x %d", width, height);
    }
  }
}

void DestroyGUI()
{
  if (h_dlg)
    DestroyWindow(h_dlg);
}

#endif

bool is_gui_available()
{
#ifdef HAS_WIN_GUI_DLG
  return (h_dlg) ? true : false;
#else
  return false;
#endif
}

void post_update_gui_init()
{
#ifdef HAS_WIN_GUI_DLG
  if (h_dlg)
    PostMessage(h_dlg, WM_PRINT, (WPARAM)0, (LPARAM)PRF_CLIENT);
#endif
}

void post_update_gui_fields()
{
#ifdef HAS_WIN_GUI_DLG
  PostMessage(h_dlg, WM_USER + 42, (WPARAM)0, (LPARAM)0);
#endif
}

void gui_SetSrate(int srate_idx)
{
#ifdef HAS_WIN_GUI_DLG
  if (h_dlg)
  {
    HWND hDlgItmSampRate = GetDlgItem(h_dlg, IDC_SAMPLERATE);
    ComboBox_SetCurSel(hDlgItmSampRate, srate_idx);
  }
#endif
}


void gui_SetAttenuator(int atten_idx)
{
#ifdef HAS_WIN_GUI_DLG
  int pos = rf_gains[atten_idx];
  if (h_dlg)
  {
    HWND hDlgItmRF_AGC = GetDlgItem(h_dlg, IDC_TUNER_RF_AGC);
    HWND hRFGain = GetDlgItem(h_dlg, IDC_RF_GAIN_SLIDER);
    HWND hRFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_RF_GAIN_VALUE);
    SendMessage(hRFGain, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)-pos);
    if (Button_GetCheck(hDlgItmRF_AGC) == BST_UNCHECKED)
    {
      TCHAR str[255];
      _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
      Static_SetText(hRFGainLabel, str);
    }
  }
#endif
}


void gui_SetMGC(int mgc_idx)
{
#ifdef HAS_WIN_GUI_DLG
  int pos = if_gains[mgc_idx];

  if (h_dlg && isR82XX())
  {
    HWND hDlgItmIF_AGC = GetDlgItem(h_dlg, IDC_TUNER_IF_AGC);
    HWND hIFGain = GetDlgItem(h_dlg, IDC_IF_GAIN_SLIDER);
    HWND hIFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_IF_GAIN_VALUE);
    SendMessage(hIFGain, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)-pos);
    if (Button_GetCheck(hDlgItmIF_AGC) == BST_UNCHECKED)
    {
      TCHAR str[255];
      _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
      Static_SetText(hIFGainLabel, str);
    }
  }
#endif
}



void DisableGUIControlsAtStart()
{
#ifdef HAS_WIN_GUI_DLG
  if (h_dlg)
  {
    BOOL en = ThreadStreamToSDR.load() ? FALSE : TRUE;
    HWND hDlgItmDevices = GetDlgItem(h_dlg, IDC_SOURCE);
    HWND hDlgItmSampMode = GetDlgItem(h_dlg, IDC_DIRECT);
    HWND hDlgItmBuffer = GetDlgItem(h_dlg, IDC_BUFFER);
    EnableWindow(hDlgItmDevices, en);
    EnableWindow(hDlgItmSampMode, en);
    EnableWindow(hDlgItmBuffer, en);
  }
#endif
}

void EnableGUIControlsAtStop()
{
  DisableGUIControlsAtStart();
}


extern "C"
void LIBRTL_API EXTIO_CALL ShowGUI()
{
#ifdef HAS_WIN_GUI_DLG
  if (h_dlg)
  {
    ShowWindow(h_dlg, SW_SHOW);
    SetForegroundWindow(h_dlg);
  }
#endif
}

extern "C"
void LIBRTL_API  EXTIO_CALL HideGUI()
{
#ifdef HAS_WIN_GUI_DLG
  if (h_dlg)
    ShowWindow(h_dlg, SW_HIDE);
#endif
}

extern "C"
void LIBRTL_API  EXTIO_CALL SwitchGUI()
{
#ifdef HAS_WIN_GUI_DLG
  if (h_dlg)
  {
    if (IsWindowVisible(h_dlg))
      ShowWindow(h_dlg, SW_HIDE);
    else
      ShowWindow(h_dlg, SW_SHOW);
  }
#endif
}

void gui_show()
{
  ShowGUI();
}


void gui_show_missing_device(int from)  // 0 == OpenHW(), 1 == StartHW()
{
#ifdef HAS_WIN_GUI_DLG
  static bool shown_openhw_err = false;

  if (!shown_openhw_err)
  {
    ::MessageBoxA(NULL, "No compatible RTL-SDR device found!", "Error", 0);
    shown_openhw_err = true;
    return;
  }

  if (from == 1 && h_dlg && !IsWindowVisible(h_dlg))
    ::MessageBoxA(NULL, "No compatible RTL-SDR device found!", "Error", 0);
#endif
}


void gui_show_invalid_device()
{
#ifdef HAS_WIN_GUI_DLG
  ::MessageBoxA(NULL, "Device invalid. Select new RTL-SDR device!", "Error", 0);
  return;
#endif
}


#ifdef HAS_WIN_GUI_DLG

static void updateTunerBWs(HWND h_dlg)
{
  TCHAR str[256];
  HWND hDlgItmTunerBW = GetDlgItem(h_dlg, IDC_TUNER_BANDWIDTH);

  bandwidths = tuners::bws[tunerNo].bw;
  n_bandwidths = tuners::bws[tunerNo].num;

  ComboBox_ResetContent(hDlgItmTunerBW);
  if (n_bandwidths)
    ComboBox_AddString(hDlgItmTunerBW, TEXT("Automatic"));
  for (int i = 1; i < n_bandwidths; i++)
  {
    _stprintf_s(str, 255, TEXT("~ %d kHz%s"), bandwidths[i], ((bandwidths[i] * 1000 > rates::MAX) ? " !" : ""));
    ComboBox_AddString(hDlgItmTunerBW, str);
  }
  ComboBox_SetCurSel(hDlgItmTunerBW, nearestBwIdx(nxt.tuner_bw));
}

static void updateTunerBandCenters(HWND h_dlg)
{
  HWND hDlgItmBandCtr = GetDlgItem(h_dlg, IDC_BAND_CENTER_SEL);
  ComboBox_ResetContent(hDlgItmBandCtr);
  ComboBox_AddString(hDlgItmBandCtr, TEXT("DC / Center: 0"));
  ComboBox_AddString(hDlgItmBandCtr, TEXT("Upper Half: + Samplerate/4"));
  ComboBox_AddString(hDlgItmBandCtr, TEXT("Lower Half: - Samplerate/4"));
  ComboBox_SetCurSel(hDlgItmBandCtr, nxt.band_center_sel);
  EnableWindow(hDlgItmBandCtr, isR82XX() ? TRUE : FALSE);
}

static void updateRFTunerGains(HWND h_dlg)
{
  HWND hRFGain = GetDlgItem(h_dlg, IDC_RF_GAIN_SLIDER);
  HWND hRFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_RF_GAIN_VALUE);

  rf_gains = tuners::rf_gains[tunerNo].gain;
  n_rf_gains = tuners::rf_gains[tunerNo].num;

  if (n_rf_gains > 0)
  {
    SendMessage(hRFGain, TBM_SETRANGEMIN, (WPARAM)TRUE, (LPARAM)-rf_gains[n_rf_gains - 1]);
    SendMessage(hRFGain, TBM_SETRANGEMAX, (WPARAM)TRUE, (LPARAM)-rf_gains[0]);
  }
  else
  {
    SendMessage(hRFGain, TBM_SETRANGEMIN, (WPARAM)TRUE, (LPARAM)-100);
    SendMessage(hRFGain, TBM_SETRANGEMAX, (WPARAM)TRUE, (LPARAM)0);
  }

  SendMessage(hRFGain, TBM_CLEARTICS, (WPARAM)FALSE, (LPARAM)0);
  if (n_rf_gains > 0)
  {
    for (int i = 0; i < n_rf_gains; i++)
      SendMessage(hRFGain, TBM_SETTIC, (WPARAM)0, (LPARAM)-rf_gains[i]);

    int gainIdx = nearestGainIdx(nxt.rf_gain, rf_gains, n_rf_gains);
    if (nxt.rf_gain != rf_gains[gainIdx])
    {
      nxt.rf_gain = rf_gains[gainIdx];
      trigger_control(CtrlFlags::rf_gain);
    }
    SendMessage(hRFGain, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)-nxt.rf_gain);
  }

  if (nxt.tuner_rf_agc)
  {
    EnableWindow(hRFGain, FALSE);
    Static_SetText(hRFGainLabel, TEXT("AGC"));
  }
  else
  {
    EnableWindow(hRFGain, TRUE);
    int pos = -SendMessage(hRFGain, TBM_GETPOS, (WPARAM)0, (LPARAM)0);
    TCHAR str[255];
    _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
    Static_SetText(hRFGainLabel, str);
  }
}


static void updateIFTunerGains(HWND h_dlg)
{
  HWND hIFGain = GetDlgItem(h_dlg, IDC_IF_GAIN_SLIDER);
  HWND hIFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_IF_GAIN_VALUE);

  if_gains = tuners::if_gains[tunerNo].gain;
  n_if_gains = tuners::if_gains[tunerNo].num;

  if (n_if_gains > 0)
  {
    SendMessage(hIFGain, TBM_SETRANGEMIN, (WPARAM)TRUE, (LPARAM)-if_gains[n_if_gains - 1]);
    SendMessage(hIFGain, TBM_SETRANGEMAX, (WPARAM)TRUE, (LPARAM)-if_gains[0]);
  }
  else
  {
    SendMessage(hIFGain, TBM_SETRANGEMIN, (WPARAM)TRUE, (LPARAM)-100);
    SendMessage(hIFGain, TBM_SETRANGEMAX, (WPARAM)TRUE, (LPARAM)0);
  }

  SendMessage(hIFGain, TBM_CLEARTICS, (WPARAM)FALSE, (LPARAM)0);
  if (n_if_gains > 0)
  {
    for (int i = 0; i < n_if_gains; i++)
      SendMessage(hIFGain, TBM_SETTIC, (WPARAM)0, (LPARAM)-if_gains[i]);

    int gain_idx = nearestGainIdx(nxt.if_gain_val, if_gains, n_if_gains);
    if (nxt.if_gain_val != if_gains[gain_idx])
    {
      nxt.if_gain_idx = gain_idx;
      nxt.if_gain_val = if_gains[nxt.if_gain_idx];
      trigger_control(CtrlFlags::if_agc_gain);
    }
    SendMessage(hIFGain, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)-nxt.if_gain_val);
  }

  if (!isR82XX())
  {
    EnableWindow(hIFGain, FALSE);
    Static_SetText(hIFGainLabel, TEXT(""));
  }
  else if (nxt.tuner_if_agc)
  {
    EnableWindow(hIFGain, FALSE);
    Static_SetText(hIFGainLabel, TEXT("AGC"));
  }
  else
  {
    EnableWindow(hIFGain, TRUE);
    int pos = -SendMessage(hIFGain, TBM_GETPOS, (WPARAM)0, (LPARAM)0);
    TCHAR str[255];
    _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
    Static_SetText(hIFGainLabel, str);
  }
}

static void updateDeviceList(HWND h_dlg)
{
  HWND hDlgItmDevices = GetDlgItem(h_dlg, IDC_SOURCE);
  ComboBox_ResetContent(hDlgItmDevices);

  uint32_t N = retrieve_devices();
  for (uint32_t k = 0; k < N; ++k)
  {
    ComboBox_AddString(hDlgItmDevices, RtlDeviceList[k].name);
  }

  ComboBox_SetCurSel(hDlgItmDevices, RtlSelectedDeviceIdx);
}


static void updateGPIOs(HWND h_dlg)
{
  if (tunerNo == RTLSDR_TUNER_FC0012 || tunerNo == RTLSDR_TUNER_FC2580)
  {
    // avoid RESET of Tuner: disable GPIO4
    for (int btnNo = 0; btnNo < NUM_GPIO_BUTTONS; ++btnNo)
    {
      if (GPIO_en[btnNo] && GPIO_pin[btnNo] == 4)
      {
        GPIO_en[btnNo] = 0;
        if (h_dlg)
        {
          HWND hDlgItmGPIObtn = GetDlgItem(h_dlg, IDC_GPIO_A + btnNo);
          //SendMessage(hDlgItmGPIObtn, WM_ENABLE, (WPARAM)(GPIO_en[btnNo] ? TRUE : FALSE), NULL);
          EnableWindow(hDlgItmGPIObtn, GPIO_en[btnNo] ? TRUE : FALSE);
        }
      }
    }
  }
  else
  {
    // re-enable GPIO4 with other Tuners
    for (int btnNo = 0; btnNo < NUM_GPIO_BUTTONS; ++btnNo)
    {
      if (!GPIO_en[btnNo] && GPIO_pin[btnNo] == 4)
      {
        GPIO_en[btnNo] = 1;
        if (h_dlg)
        {
          HWND hDlgItmGPIObtn = GetDlgItem(h_dlg, IDC_GPIO_A + btnNo);
          //SendMessage(hDlgItmGPIObtn, WM_ENABLE, (WPARAM)(GPIO_en[btnNo] ? TRUE : FALSE), NULL);
          EnableWindow(hDlgItmGPIObtn, GPIO_en[btnNo] ? TRUE : FALSE);
        }
      }
    }
  }
}


INT_PTR CALLBACK MainDlgProc(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  char acMsg[256];
  HWND h_dlg = hwndDlg;
  static HBRUSH BRUSH_RED = CreateSolidBrush(RGB(255, 0, 0));
  static HBRUSH BRUSH_GREEN = CreateSolidBrush(RGB(0, 255, 0));

  switch (uMsg)
  {
  case (WM_USER + 42):
  case WM_INITDIALOG:
  {
    HWND hDlgItmOffset = GetDlgItem(h_dlg, IDC_OFFSET);
    HWND hDlgItmUsbSB = GetDlgItem(h_dlg, IDC_USB_SIDEBAND);
    HWND hDlgItmBandText = GetDlgItem(h_dlg, IDC_BAND_NAME);
    HWND hDlgItmBandCtr = GetDlgItem(h_dlg, IDC_BAND_CENTER_SEL);
    HWND hDlgItmSampMode = GetDlgItem(h_dlg, IDC_DIRECT);
    HWND hDlgItmSampRate = GetDlgItem(h_dlg, IDC_SAMPLERATE);
    HWND hDlgItmRF_AGC = GetDlgItem(h_dlg, IDC_TUNER_RF_AGC);
    HWND hDlgItmIF_AGC = GetDlgItem(h_dlg, IDC_TUNER_IF_AGC);
    HWND hDlgItmRtlDig_AGC = GetDlgItem(h_dlg, IDC_RTL_DIG_AGC);
    HWND hDlgItmRtl_INC = GetDlgItem(h_dlg, IDC_RTL_INC);
    HWND hDlgItmBuffer = GetDlgItem(h_dlg, IDC_BUFFER);
    HWND hDlgItmPPM = GetDlgItem(h_dlg, IDC_PPM);
    HWND hDlgItmPPMS = GetDlgItem(h_dlg, IDC_PPM_S);

    const bool upd_band_text = update_band_text.exchange(false);
    if (upd_band_text)
      Static_SetText(hDlgItmBandText, band_disp_text);

    ComboBox_ResetContent(hDlgItmSampMode);
    for (int i = 0; i < (sizeof(s_modes) / sizeof(s_modes[0])); i++)
      ComboBox_AddString(hDlgItmSampMode, s_modes[i]);
    ComboBox_SetCurSel(hDlgItmSampMode, nxt.sampling_mode);

    updateTunerBandCenters(h_dlg);

    Button_SetCheck(hDlgItmRF_AGC, nxt.tuner_rf_agc ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(hDlgItmIF_AGC, (nxt.tuner_if_agc && isR82XX()) ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(hDlgItmRtlDig_AGC, nxt.rtl_agc ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(hDlgItmOffset, nxt.offset_tuning ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(hDlgItmUsbSB, nxt.USB_sideband ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(hDlgItmRtl_INC, nxt.rtl_impulse_noise_cancellation ? BST_CHECKED : BST_UNCHECKED);

    ShowWindow(hDlgItmOffset, (RTLSDR_TUNER_E4000 == tunerNo) ? SW_SHOW : SW_HIDE);
    ShowWindow(hDlgItmUsbSB, isR82XX() ? SW_SHOW : SW_HIDE);
    EnableWindow(hDlgItmIF_AGC, isR82XX() ? TRUE : FALSE);
    EnableWindow(hDlgItmBandCtr, isR82XX() ? TRUE : FALSE);

    TCHAR tempStr[256];

    for (int btnNo = 0; btnNo < NUM_GPIO_BUTTONS; ++btnNo)
    {
      HWND hDlgItmGPIObtn = GetDlgItem(h_dlg, IDC_GPIO_A + btnNo);

      _stprintf_s(tempStr, 255, TEXT("%s"), &GPIO_txt[btnNo][0]);
      Edit_SetText(hDlgItmGPIObtn, tempStr);

      if (GPIO_en[btnNo])
      {
        EnableWindow(hDlgItmGPIObtn, TRUE);
        Button_SetCheck(hDlgItmGPIObtn, nxt.GPIO[btnNo] ? BST_CHECKED : BST_UNCHECKED);
      }
      else
      {
        EnableWindow(hDlgItmGPIObtn, FALSE);
        Button_SetCheck(hDlgItmGPIObtn, BST_UNCHECKED);
      }
    }

    SendMessage(hDlgItmPPMS, UDM_SETRANGE, (WPARAM)TRUE, (LPARAM)MAX_PPM | (MIN_PPM << 16));

    _stprintf_s(tempStr, 255, TEXT("%d"), nxt.freq_corr_ppm.load());
    Edit_SetText(hDlgItmPPM, tempStr);
    //rtlsdr_set_freq_correction(dev, nxt.freq_corr_ppm);

    ComboBox_ResetContent(hDlgItmSampRate);
    for (int i = 0; i < rates::N; i++)
      ComboBox_AddString(hDlgItmSampRate, rates::tab[i].name);
    ComboBox_SetCurSel(hDlgItmSampRate, nxt.srate_idx);

    {
      for (int i = 0; i < (sizeof(buffer_sizes) / sizeof(buffer_sizes[0])); i++)
      {
        TCHAR str[256];
        _stprintf_s(str, 256, TEXT("%d kB"), buffer_sizes[i]);
        ComboBox_AddString(hDlgItmBuffer, str);
      }
      ComboBox_SetCurSel(hDlgItmBuffer, bufferSizeIdx);
      buffer_len = buffer_sizes[bufferSizeIdx] * 1024;
    }

    updateTunerBWs(h_dlg);
    updateRFTunerGains(h_dlg);
    updateIFTunerGains(h_dlg);
    updateGPIOs(h_dlg);
    updateDeviceList(h_dlg);

    return TRUE;
  }

  case WM_PRINT:
    if (lParam == (LPARAM)PRF_CLIENT)
    {
      HWND hDlgItmDevices = GetDlgItem(h_dlg, IDC_SOURCE);
      HWND hDlgItmOffset = GetDlgItem(h_dlg, IDC_OFFSET);
      HWND hDlgItmUsbSB = GetDlgItem(h_dlg, IDC_USB_SIDEBAND);
      HWND hDlgItmBandText = GetDlgItem(h_dlg, IDC_BAND_NAME);
      HWND hDlgItmTunerBW = GetDlgItem(h_dlg, IDC_TUNER_BANDWIDTH);
      HWND hDlgItmBandCenter = GetDlgItem(h_dlg, IDC_BAND_CENTER_SEL);
      HWND hIFGain = GetDlgItem(h_dlg, IDC_IF_GAIN_SLIDER);
      HWND hDlgItmIF_AGC = GetDlgItem(h_dlg, IDC_TUNER_IF_AGC);
      HWND hDlgItmTunerModel = GetDlgItem(h_dlg, IDC_TUNER_MODEL);

      updateTunerBWs(h_dlg);
      updateRFTunerGains(h_dlg);
      updateIFTunerGains(h_dlg);
      updateGPIOs(h_dlg);
      updateDeviceList(h_dlg);

      BOOL enableOffset = (RTLSDR_TUNER_E4000 == tunerNo) ? TRUE : FALSE;
      BOOL enableSideBand = (isR82XX()) ? TRUE : FALSE;
      BOOL enableBandCenter = (isR82XX()) ? TRUE : FALSE;
      BOOL enableIFGain = (!nxt.tuner_if_agc && isR82XX()) ? TRUE : FALSE;
      BOOL enableTunerBW = (bandwidths && 0 == nxt.sampling_mode) ? TRUE : FALSE;
      BOOL enableDeviceList = (!RtlSdrDev || !ThreadStreamToSDR.load());

      EnableWindow(hDlgItmDevices, enableDeviceList);
      EnableWindow(hDlgItmOffset, enableOffset);
      EnableWindow(hDlgItmUsbSB, enableSideBand);
      EnableWindow(hDlgItmTunerBW, enableTunerBW);
      EnableWindow(hDlgItmBandCenter, enableBandCenter);
      EnableWindow(hIFGain, enableIFGain);
      EnableWindow(hDlgItmIF_AGC, isR82XX() ? TRUE : FALSE);

      ShowWindow(hDlgItmOffset, (RTLSDR_TUNER_E4000 == tunerNo) ? SW_SHOW : SW_HIDE);
      ShowWindow(hDlgItmUsbSB, isR82XX() ? SW_SHOW : SW_HIDE);

      const char* tunerText = tuners::names[tunerNo];
      TCHAR str[255];
      _stprintf_s(str, 255, TEXT("Tuner: %s"), tunerText);
      Static_SetText(hDlgItmTunerModel, str);

      const bool upd_band_text = update_band_text.exchange(false);
      if (upd_band_text)
        Static_SetText(hDlgItmBandText, band_disp_text);
    }
    return TRUE;

  case WM_COMMAND:
    switch (GET_WM_COMMAND_ID(wParam, lParam))
    {
    case IDC_PPM:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == EN_CHANGE)
      {
        TCHAR ppm[255];
        Edit_GetText((HWND)lParam, ppm, 255);
        nxt.freq_corr_ppm = _ttoi(ppm);
        trigger_control(CtrlFlags::ppm_correction);
        EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_LO);
      }
      return TRUE;
    case IDC_RTL_DIG_AGC:
    {
      nxt.rtl_agc = (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) ? 1 : 0;
      trigger_control(CtrlFlags::rtl_agc);
      return TRUE;
    }
    case IDC_RTL_INC:
    {
      int nc = (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) ? 1 : 0;
      nxt.rtl_impulse_noise_cancellation = nc;
      trigger_control(CtrlFlags::rtl_impulse_nc);
      return TRUE;
    }
    case IDC_OFFSET:
    {
      HWND hDlgItmOffset = GetDlgItem(h_dlg, IDC_OFFSET);

      nxt.offset_tuning = (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) ? 1 : 0;
      trigger_control(CtrlFlags::offset_tuning);

      BOOL enableOffset = (RTLSDR_TUNER_E4000 == tunerNo) ? TRUE : FALSE;
      EnableWindow(hDlgItmOffset, enableOffset);
      return TRUE;
    }
    case IDC_USB_SIDEBAND:
    {
      HWND hDlgItmUsbSB = GetDlgItem(h_dlg, IDC_USB_SIDEBAND);

      nxt.USB_sideband = (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) ? 1 : 0;
      trigger_control(CtrlFlags::tuner_sideband);

      BOOL enableSideBand = (isR82XX()) ? TRUE : FALSE;
      EnableWindow(hDlgItmUsbSB, enableSideBand);
      return TRUE;
    }
    case IDC_GPIO_A:
    case IDC_GPIO_B:
    case IDC_GPIO_C:
    case IDC_GPIO_D:
    case IDC_GPIO_E:
    {
      const int btnNo = GET_WM_COMMAND_ID(wParam, lParam) - IDC_GPIO_A;
      HWND hDlgItmGPIObtn = GetDlgItem(h_dlg, IDC_GPIO_A + btnNo);
      if (GPIO_en[btnNo])
      {
        nxt.GPIO[btnNo] = (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) ? 1 : 0;
        trigger_control(CtrlFlags::gpio);
      }
      EnableWindow(hDlgItmGPIObtn, GPIO_en[btnNo] ? TRUE : FALSE);
      return TRUE;
    }

    case IDC_TUNER_RF_AGC:
    {
      HWND hRFGain = GetDlgItem(h_dlg, IDC_RF_GAIN_SLIDER);
      HWND hRFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_RF_GAIN_VALUE);
      if (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) //it is checked
      {
        nxt.tuner_rf_agc = 1; // automatic
        trigger_control(CtrlFlags::rf_agc);
        EnableWindow(hRFGain, FALSE);
        Static_SetText(hRFGainLabel, TEXT("HF AGC"));
      }
      else //it has been unchecked
      {
        //rtlsdr_set_tuner_gain_mode(dev,1);
        nxt.tuner_rf_agc = 0; // manual
        trigger_control(CtrlFlags::rf_agc);
        EnableWindow(hRFGain, TRUE);
        int pos = -SendMessage(hRFGain, TBM_GETPOS, (WPARAM)0, (LPARAM)0);
        TCHAR str[255];
        _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
        Static_SetText(hRFGainLabel, str);
      }
      EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_RF_IF);  // Signal application
      return TRUE;
    }

    case IDC_TUNER_IF_AGC:
    {
      HWND hIFGain = GetDlgItem(h_dlg, IDC_IF_GAIN_SLIDER);
      HWND hIFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_IF_GAIN_VALUE);
      if (!isR82XX())
      {
        EnableWindow(hIFGain, FALSE);
        Static_SetText(hIFGainLabel, TEXT(""));
      }
      else if (Button_GetCheck(GET_WM_COMMAND_HWND(wParam, lParam)) == BST_CHECKED) //it is checked
      {
        nxt.tuner_if_agc = 1; // automatic
        last.if_gain_val = nxt.if_gain_val + 1;
        last.if_gain_idx = nxt.if_gain_idx + 1;
        trigger_control(CtrlFlags::if_agc_gain);
        EnableWindow(hIFGain, FALSE);
        Static_SetText(hIFGainLabel, TEXT("IF AGC"));
      }
      else //it has been unchecked
      {
        nxt.tuner_if_agc = 0; // manual
        last.if_gain_val = nxt.if_gain_val + 1;
        last.if_gain_idx = nxt.if_gain_idx + 1;
        trigger_control(CtrlFlags::if_agc_gain);
        BOOL enableIFGain = (isR82XX()) ? TRUE : FALSE;
        EnableWindow(hIFGain, enableIFGain);
        int pos = -SendMessage(hIFGain, TBM_GETPOS, (WPARAM)0, (LPARAM)0);
        TCHAR str[255];
        _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
        Static_SetText(hIFGainLabel, str);
      }

      EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_RF_IF);  // Signal application
      return TRUE;
    }

    case IDC_SAMPLERATE:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE)
      {
        nxt.srate_idx = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        trigger_control(CtrlFlags::srate);
        EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_SampleRate);  // Signal application
      }
      if (0 && GET_WM_COMMAND_CMD(wParam, lParam) == CBN_EDITUPDATE)
      {
        TCHAR  ListItem[256];
        ComboBox_GetText((HWND)lParam, ListItem, 256);
        TCHAR* endptr;
        double coeff = _tcstod(ListItem, &endptr);
        while (_istspace(*endptr)) ++endptr;

        int exp = 1;
        switch (_totupper(*endptr)) {
        case 'K': exp = 1000; break;
        case 'M': exp = 1000 * 1000; break;
        }

        uint32_t newrate = uint32_t(coeff * exp);
        if (newrate >= rates::MIN && newrate <= rates::MAX) {
          //rtlsdr_set_sample_rate(dev, newrate);
// @TODO!
          EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_SampleRate);  // Signal application
        }

      }
      return TRUE;

    case IDC_BUFFER:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE)
      {
        bufferSizeIdx = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        buffer_len = buffer_sizes[bufferSizeIdx] * 1024;
        EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_SampleRate);  // Signal application
        //if (SDRsupportsLogging)
        //  SDRLOG(extHw_MSG_ERRDLG, "Restart SDR application,\nthat changed buffer size has effect!");
        //else
        //  ::MessageBoxA(NULL, "Restart SDR application,\nthat changed buffer size has effect!", "Info", 0);
      }
      return TRUE;

    case IDC_TUNER_BANDWIDTH:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE)
      {
        int bwIdx = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        nxt.tuner_bw = bandwidths[bwIdx];
        trigger_control(CtrlFlags::tuner_bandwidth);
      }
      return TRUE;

    case IDC_DIRECT:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE)
      {
        nxt.sampling_mode = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        trigger_control(CtrlFlags::sampling_mode);

        EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_LO);  // Signal application
      }
      return TRUE;

    case IDC_BAND_CENTER_SEL:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE)
      {
        const int64_t prev_tuneFreq = nxt.tune_freq;
        int64_t tmp_LO_freq = nxt.LO_freq;
        const int fs = rates::tab[nxt.srate_idx].valueInt;
        nxt.band_center_sel = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        int band_center = 0;
        if (nxt.band_center_sel == 1)
          band_center = fs / 4;
        else if (nxt.band_center_sel == 2)
          band_center = -fs / 4;
        nxt.band_center_LO_delta = band_center;

        if (last.band_center_sel == 1)  // -> +fs/4
          tmp_LO_freq += fs / 4;
        else if (last.band_center_sel == 2)  // -> -fs/4
          tmp_LO_freq -= fs / 4;

        if (nxt.band_center_sel == 1)  // -> +fs/4
          tmp_LO_freq -= fs / 4;
        else if (nxt.band_center_sel == 2)  // -> -fs/4
          tmp_LO_freq += fs / 4;

        nxt.LO_freq = tmp_LO_freq;
        trigger_control(CtrlFlags::tuner_band_center | CtrlFlags::freq);

        EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_LO);

        // looks, we need to delay tuning back to previous frequency!
        retune_value = prev_tuneFreq;
        retune_counter = fs / 10;
        retune_freq.store(true);

        BOOL enableBandCenter = (isR82XX()) ? TRUE : FALSE;
        HWND hDlgItmBandCenter = GetDlgItem(h_dlg, IDC_BAND_CENTER_SEL);
        EnableWindow(hDlgItmBandCenter, enableBandCenter);
      }
      return TRUE;

    case IDC_SOURCE:
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE)
      {
        RtlSelectedDeviceIdx = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        if (RtlSelectedDeviceIdx >= RtlNumDevices)
          RtlSelectedDeviceIdx = 0;
        if (RtlSelectedDeviceIdx >= MAX_RTL_DEVICES
          || RtlDeviceList[RtlSelectedDeviceIdx].dev_idx >= MAX_RTL_DEVICES)
        {
          SDRLG(extHw_MSG_ERROR, "Source ComboBox selected invalid device %u of %u with dev_idx %u",
            RtlSelectedDeviceIdx, RtlNumDevices, RtlDeviceList[RtlSelectedDeviceIdx].dev_idx);
        }
        else if (RtlSdrDev && !ThreadStreamToSDR.load())
          // && !RtlDeviceInfo::is_same(RtlOpenDevice, RtlDeviceList[RtlSelectedDeviceIdx]))
        {
          open_selected_rtl_device();
          post_update_gui_init();  // post_update_gui_fields();
        }
      }
      if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_DROPDOWN)
      {
        updateDeviceList(h_dlg);
      }
      return TRUE;
    }
    break;

  case WM_VSCROLL:
  {
    HWND hRFGain = GetDlgItem(h_dlg, IDC_RF_GAIN_SLIDER);
    HWND hRFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_RF_GAIN_VALUE);
    if ((HWND)lParam == hRFGain)
    {
      int pos = -SendMessage(hRFGain, TBM_GETPOS, (WPARAM)0, (LPARAM)0);
      for (int i = 0; i < n_rf_gains - 1; ++i)
        if (rf_gains[i] < pos && pos < rf_gains[i + 1])
        {
          if ((pos - rf_gains[i]) < (rf_gains[i + 1] - pos) && (LOWORD(wParam) != TB_LINEUP) || (LOWORD(wParam) == TB_LINEDOWN))
            pos = rf_gains[i];
          else
            pos = rf_gains[i + 1];
        }

      SendMessage(hRFGain, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)-pos);
      TCHAR str[255];
      _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
      Static_SetText(hRFGainLabel, str);

      if (pos != last.rf_gain)
      {
        nxt.rf_gain = pos;
        trigger_control(CtrlFlags::rf_gain);
        EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_ATT);
      }
      return TRUE;
    }

    HWND hIFGain = GetDlgItem(h_dlg, IDC_IF_GAIN_SLIDER);
    HWND hIFGainLabel = GetDlgItem(h_dlg, IDC_TUNER_IF_GAIN_VALUE);
    if ((HWND)lParam == hIFGain && isR82XX())
    {
      int pos = -SendMessage(hIFGain, TBM_GETPOS, (WPARAM)0, (LPARAM)0);
      for (int i = 0; i < n_if_gains - 1; ++i)
        if (if_gains[i] < pos && pos < if_gains[i + 1])
        {
          if ((pos - if_gains[i]) < (if_gains[i + 1] - pos) && (LOWORD(wParam) != TB_LINEUP) || (LOWORD(wParam) == TB_LINEDOWN))
            pos = if_gains[i];
          else
            pos = if_gains[i + 1];
        }

      SendMessage(hIFGain, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)-pos);
      TCHAR str[255];
      _stprintf_s(str, 255, TEXT("%2.1f dB"), (float)pos / 10);
      Static_SetText(hIFGainLabel, str);

      if (pos != last.if_gain_val)
      {
        nxt.if_gain_val = pos;
        nxt.if_gain_idx = nearestGainIdx(nxt.if_gain_val, if_gains, n_if_gains);
        trigger_control(CtrlFlags::if_agc_gain);
        //EXTIO_STATUS_CHANGE(gpfnExtIOCallbackPtr, extHw_Changed_ATT);
      }
      return TRUE;
    }

    HWND hDlgItmPPMS = GetDlgItem(h_dlg, IDC_PPM_S);
    if ((HWND)lParam == hDlgItmPPMS)
    {
      return TRUE;
    }
  }
  break;

  case WM_CLOSE:
    ShowWindow(h_dlg, SW_HIDE);
    return TRUE;
    break;

  case WM_DESTROY:
    ::h_dlg = NULL;
    return TRUE;
    break;

    /*
    * TODO: Add more messages, when needed.
    */
  }

  return FALSE;
}

#endif
