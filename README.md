## ExtIO_RTL

ExtIO plugin for use with [**HDSDR**](https://hdsdr.de/) and other [**Winrad**](https://www.i2phd.org/winrad/) compatible programs on Windows, e.g. [**SDRuno**](https://www.sdrplay.com/sdruno/).

This ExtIO plugin is a wrapper around [**librtlsdr**](https://github.com/hayguen/librtlsdr), the main driver for RTL-SDR receiver devices (aka dongles).


### Setup:

Simply put/copy the `ExtIO_RTL.dll` alongside to the SDR executable for HDSDR.
Put/copy the `ExtIO_RTL.dll` into your 'Documents' folder for SDRuno.
This `dll` is in the `ExtIO_RTL_x32_v....zip` file located at https://github.com/hayguen/ExtIO_RTL/releases, which has to be downloaded and unpacked first.

[Zadig](https://zadig.akeo.ie/), the USB driver has to be installed and configured for the RTL device(s).

The ExtIO dll is a static build - including librtlsdr. Other DLLs or dependencies are *NOT* necessary.


### Additional features with v2023 and later releases

* enhanced features for R860 or R820T/2 and also R828D tuners:
  - Narrow tuner bandwidths - downto ~ 300 kHz - improving dynamic and 'sensivity' by removing unnecessary signals
  - Band Center configuration to get rid of the received DC from the 'signal of interest'
  - Tune USB band: mirroring the spectrum - switches the band corner, where the steeper low pass filter is applied
  - RF gain: controls LNA and mixer
  - IF gain: controls VGA - with feedback from RTL2832U in case of enabled AGC
  - supports the RTL-SDR blog V4 dongle - based on the R828D tuner and including an HF upconverter. see  https://www.rtl-sdr.com/rtl-sdr-blog-v4-dongle-initial-release/
  - supports the RTL-SDR blog V4 Lite dongle - based on the R828S tuner and including an HF upconverter
  - note: keep the IF-AGC / Auto-VGA off for R828D based tuners: it often produces 'pumping'
* explicit Bias Tee and GPIO controls in the GUI
* band configurations enabled by editing rtl_sdr_extio.cfg in your %USERPROFILE% directory
  - default/demo configuration is generated at first start
  - it's disabled by default
  - should be self-explanatory with comment lines
* control Impulse Noise Cancellation function of RTL2832U. this sound very interesting - especially on HF frequencies


### Known issue(s)

* the comboboxes' list popup doesn't show up on some SDR programs
  - use mouse-wheel or cursor up/down keys to select other available entries, e.g. for tuner bandwidth


### History / Source

* Original code from [**jorgem-seq/ExtIO_RTL**](https://github.com/jorgem-seq/ExtIO_RTL), which was derived from [**josemariaaraujo/ExtIO_RTL**](https://github.com/josemariaaraujo/ExtIO_RTL)
* refactored a lot and took much from [extio_rtl_tcp](https://github.com/hayguen/extio_rtl_tcp)
* removed makefiles and added cmake support to compile with Visual Studio 2019
* added all required libraries as submodules
* added github action for automatic build

Clone with submodules to automatically retrieve libusb, pthread-win32 and librtlsdr from git bash:

```
git clone -b cmake_github_action --recursive https://github.com/hayguen/ExtIO_RTL.git
```

Manual compilation from Visual Studio 2019 Command Prompt:
```
cmake -G "Visual Studio 16 2019" -A Win32 -S ExtIO_RTL -B build_ExtIO_RTL
cmake --build build_ExtIO_RTL --config Release --target ExtIO_RTL
cmake --build build_ExtIO_RTL --config Release --target rtl_tools
```

A pre-built binary should be available in github Actions.

### LICENSE

GPL-2.0, see [**LICENSE file**](COPYING)
