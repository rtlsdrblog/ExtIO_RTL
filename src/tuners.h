#pragma once

struct tuners
{

  struct gain_t
  {
    const int* gain;  // 0.1 dB steps: gain in dB = gain[] / 10
    const int num;
  };

  struct bw_t
  {
    const int* bw;  // bw in kHz: bw in Hz = bw[] * 1000
    const int num;
  };

  static constexpr unsigned N = 9;
  static const char* names[N];

  static const gain_t rf_gains[N];
  static const gain_t if_gains[N];
  static const bw_t bws[N];

};
