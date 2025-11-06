#pragma once

#include <stdint.h>

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef pgm_read_float
#define pgm_read_float(addr) (*(const float *)(addr))
#endif

#ifndef pgm_read_ptr
#define pgm_read_ptr(addr) (*(const void *const *)(addr))
#endif

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

struct VsopTerm {
  float amplitude;
  float phase;
  float frequency;
};

struct VsopComponent {
  const VsopTerm *terms;
  uint8_t count;
};

struct PlanetTerms {
  VsopComponent x;
  VsopComponent y;
  VsopComponent z;
};

extern const PlanetTerms kPlanetTerms[] PROGMEM;

