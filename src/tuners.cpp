
#include "tuners.h"

static const int e4k_gains[] =
{ -10, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420 };

static const int fc12_gains[] = { -99, -40, 71, 179, 192 };

static const int fc13_gains[] =
{ -99, -73, -65, -63, -60, -58, -54, 58, 61, 63, 65, 67, 68, 70, 71, 179, 181, 182, 184, 186, 188, 191, 197 };

static const int r820t_gains[] =
{ 0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496 };

static const int r820t_if_gains[] =
{ -47, -21, 5, 35, 77, 112, 136, 149, 163, 195, 231, 265, 300, 337, 372, 408 };

// mix: 1900, 2300, 2700, 3300, 3400,
// rc: 1000, 1200, 1800, 2600, 3400,
// if channel: 2150, 2200, 2240, 2280, 2300, 2400, 2450, 2500, 2550, 2600, 2700, 2750, 2800, 2900, 2950, 3000, 3100, 3200, 3300, 3400
static const int e4k_bws[] =
{ 0, 1000, 1200, 1800, 1900, 2150, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400,   5000, 10000 };

static const int r820_bws[] =
//{ 0, 350, 450, 550, 700, 900, 1200, 1450, 1550, 1600, 1700, 1800, 1900, 1950, 2050, 2080, 2180, 2280, 2330, 2430, 6000, 7000, 8000 };
//{ 0, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 6000, 7000, 8000 };
//{ 0, 200, 201, 300, 301, 400, 401, 500, 501, 600, 601, 700, 701, 800, 801, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 6000, 7000, 8000 };
//{ 0, 201, 202, 290, 301, 302, 375, 401, 402, 420, 470, 501, 502, 600, 601, 602, 701, 702, 801, 802, 860, 901, 950, 1001, 1100, 1101, 1201, 1300, 1301, 1503, 1600, 1753, 1953, 6000, 7000, 8000 };
//{ 0, 202, 290, 302, 375, 420, 470, 502, 600, 602, 701, 702, 801, 802, 860, 901, 950, 1001, 1100, 1101, 1201, 1300, 1301, 1503, 1600, 1753, 1953, 6000, 7000, 8000 };
//{ 0, 290, 375, 420, 470, 600, 860, 950, 1100, 1300, 1500, 1600, 1750, 1950, 6000 /*, 7000, 8000 */ };
{ 0, 290, 375, 420, 470, 600, 860, 950, 1100, 1200, 1300, 1500, 1600, 1750, 1800, 1950, 2200, 3000, 5000 };


const char* tuners::names[tuners::N] = {
  "UNKNOWN"
, "E4000"
, "FC0012"
, "FC0013"
, "FC2580"
, "R860 or R820T/2"
, "R828D"
, "BLOG V4 with R828D"
, "BLOG V4 Lite with R828S"
};

const tuners::gain_t tuners::rf_gains[tuners::N] =
{
  { 0, 0 }  // tuner_type: E4000 =1, FC0012 =2, FC0013 =3, FC2580 =4, R820T =5, R828D =6
, { e4k_gains, sizeof(e4k_gains) / sizeof(e4k_gains[0]) }
, { fc12_gains, sizeof(fc12_gains) / sizeof(fc12_gains[0]) }
, { fc13_gains, sizeof(fc13_gains) / sizeof(fc13_gains[0]) }
, { 0, 0 }  // FC2580
, { r820t_gains, sizeof(r820t_gains) / sizeof(r820t_gains[0]) }  // R820T/2
, { r820t_gains, sizeof(r820t_gains) / sizeof(r820t_gains[0]) }  // R828D
, { r820t_gains, sizeof(r820t_gains) / sizeof(r820t_gains[0]) }  // RTLSDR_TUNER_BLOG_V4
, { r820t_gains, sizeof(r820t_gains) / sizeof(r820t_gains[0]) }  // RTLSDR_TUNER_BLOG_V4L
};

const tuners::gain_t tuners::if_gains[tuners::N] =
{
  { 0, 0 }  // tuner_type: E4000 =1, FC0012 =2, FC0013 =3, FC2580 =4, R820T =5, R828D =6, BLOG_V4=7
, { 0, 0 }  // E4000
, { 0, 0 }  // FC012
, { 0, 0 }  // FC013
, { 0, 0 }  // FC2580
, { r820t_if_gains, sizeof(r820t_if_gains) / sizeof(r820t_if_gains[0]) }  // R820T/2
, { r820t_if_gains, sizeof(r820t_if_gains) / sizeof(r820t_if_gains[0]) }  // R828D
, { r820t_if_gains, sizeof(r820t_if_gains) / sizeof(r820t_if_gains[0]) }  // RTLSDR_TUNER_BLOG_V4
, { r820t_if_gains, sizeof(r820t_if_gains) / sizeof(r820t_if_gains[0]) }  // RTLSDR_TUNER_BLOG_V4L
};


const tuners::bw_t tuners::bws[] =
{
  { 0, 0 }  // tuner_type: E4000 =1, FC0012 =2, FC0013 =3, FC2580 =4, R820T =5, R828D =6
, { e4k_bws, sizeof(e4k_bws) / sizeof(e4k_bws[0]) }
, { 0, 0 }  // FC0012
, { 0, 0 }  // FC0013
, { 0, 0 }  // FC2580
, { r820_bws, sizeof(r820_bws) / sizeof(r820_bws[0]) }  // R820T/2
, { r820_bws, sizeof(r820_bws) / sizeof(r820_bws[0]) }  // R828D
, { r820_bws, sizeof(r820_bws) / sizeof(r820_bws[0]) }  // RTLSDR_TUNER_BLOG_V4
, { r820_bws, sizeof(r820_bws) / sizeof(r820_bws[0]) }  // RTLSDR_TUNER_BLOG_V4L
};
