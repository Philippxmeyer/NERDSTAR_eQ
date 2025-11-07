#include "planets.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef ARDUINO
#include <avr/pgmspace.h>
#endif

#include "vsop87a_data.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToHour = 12.0f / kPi;
constexpr float kJulianCentury = 36525.0f;
constexpr float kJulianMillennium = 365250.0f;
constexpr float kSpeedOfLightAuPerDay = 173.1446327f;
constexpr float kObliquityJ2000 = 23.43929111f * kDegToRad;

struct Vec3 {
  float x;
  float y;
  float z;
};

float normalizeRadians(float value) {
  float result = fmodf(value, kTwoPi);
  if (result < 0.0f) {
    result += kTwoPi;
  }
  return result;
}

float julianCenturies(float jd) {
  return (jd - 2451545.0f) / kJulianCentury;
}

float julianMillennia(float jd) {
  return (jd - 2451545.0f) / kJulianMillennium;
}

float evaluateComponent(const VsopComponent& component, float t) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < component.count; ++i) {
    const VsopTerm* termPtr = component.terms + i;
    float amplitude = pgm_read_float(&termPtr->amplitude);
    float phase = pgm_read_float(&termPtr->phase);
    float frequency = pgm_read_float(&termPtr->frequency);
    sum += amplitude * cosf(phase + frequency * t);
  }
  return sum;
}

Vec3 heliocentricPosition(PlanetId id, float jd) {
  PlanetTerms terms;
#ifdef ARDUINO
  int index = static_cast<int>(id);
  terms.x.terms = reinterpret_cast<const VsopTerm *>(
      pgm_read_ptr(&kPlanetTerms[index].x.terms));
  terms.x.count = pgm_read_byte(&kPlanetTerms[index].x.count);
  terms.y.terms = reinterpret_cast<const VsopTerm *>(
      pgm_read_ptr(&kPlanetTerms[index].y.terms));
  terms.y.count = pgm_read_byte(&kPlanetTerms[index].y.count);
  terms.z.terms = reinterpret_cast<const VsopTerm *>(
      pgm_read_ptr(&kPlanetTerms[index].z.terms));
  terms.z.count = pgm_read_byte(&kPlanetTerms[index].z.count);
#else
  terms = kPlanetTerms[static_cast<int>(id)];
#endif
  float t = julianMillennia(jd);
  Vec3 result;
  result.x = evaluateComponent(terms.x, t);
  result.y = evaluateComponent(terms.y, t);
  result.z = evaluateComponent(terms.z, t);
  return result;
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 add(const Vec3& a, const Vec3& b) {
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 scale(const Vec3& v, float s) {
  return Vec3{v.x * s, v.y * s, v.z * s};
}

float dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

float magnitude(const Vec3& v) {
  return sqrtf(dot(v, v));
}

Vec3 unitVector(const Vec3& v) {
  float mag = magnitude(v);
  if (mag == 0.0f) {
    return Vec3{0.0f, 0.0f, 0.0f};
  }
  return scale(v, 1.0f / mag);
}

struct Matrix3 {
  float m[3][3];
};

Matrix3 rotationX(float angle) {
  float s = sinf(angle);
  float c = cosf(angle);
  Matrix3 r{};
  r.m[0][0] = 1.0f;
  r.m[1][1] = c;
  r.m[1][2] = -s;
  r.m[2][1] = s;
  r.m[2][2] = c;
  return r;
}

Matrix3 rotationZ(float angle) {
  float s = sinf(angle);
  float c = cosf(angle);
  Matrix3 r{};
  r.m[0][0] = c;
  r.m[0][1] = -s;
  r.m[1][0] = s;
  r.m[1][1] = c;
  r.m[2][2] = 1.0f;
  return r;
}

Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
  Matrix3 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] +
                       a.m[i][2] * b.m[2][j];
    }
  }
  return result;
}

Vec3 transform(const Matrix3& m, const Vec3& v) {
  return Vec3{m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
              m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
              m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z};
}

void computeNutation(float t, float& deltaPsi, float& deltaEpsilon) {
  // Fundamental arguments (IAU 2000)
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;

  float l = fmodf(485868.249036f + 1717915923.2178f * t + 31.8792f * t2 +
                      0.051635f * t3 - 0.00024470f * t4,
                  1296000.0f) * (kDegToRad / 3600.0f);
  float lp = fmodf(1287104.793048f + 129596581.0481f * t - 0.5532f * t2 +
                       0.000136f * t3 - 0.00001149f * t4,
                   1296000.0f) * (kDegToRad / 3600.0f);
  float f = fmodf(335779.526232f + 1739527262.8478f * t - 12.7512f * t2 -
                      0.001037f * t3 + 0.00000417f * t4,
                  1296000.0f) * (kDegToRad / 3600.0f);
  float d = fmodf(1072260.703692f + 1602961601.2090f * t - 6.3706f * t2 +
                      0.006593f * t3 - 0.00003169f * t4,
                  1296000.0f) * (kDegToRad / 3600.0f);
  float om = fmodf(450160.398036f - 6962890.5431f * t + 7.4722f * t2 +
                       0.007702f * t3 - 0.00005939f * t4,
                   1296000.0f) * (kDegToRad / 3600.0f);

  // Truncated IAU 2000B series (leading five terms) in 0.1 microarcseconds.
  struct NutationTerm {
    int8_t nl;
    int8_t nlp;
    int8_t nf;
    int8_t nd;
    int8_t nom;
    float sp;
    float spt;
    float ce;
    float cet;
  };

  static const NutationTerm kTerms[] PROGMEM = {
      {0, 0, 0, 0, 1, -172064.161f, -174.2f, 92053.0f, 8.9f},
      {0, 0, 2, -2, 2, -13187.0f, -1.6f, 5736.0f, -3.1f},
      {0, 0, 2, 0, 2, -2274.0f, -0.2f, 977.0f, -0.5f},
      {0, 0, 0, 0, 2, 2062.0f, 0.2f, -895.0f, 0.5f},
      {0, 1, 0, 0, 0, 1426.0f, -3.4f, 54.0f, -0.1f},
  };

  float sumPsi = 0.0f;
  float sumEps = 0.0f;
  for (uint8_t i = 0; i < sizeof(kTerms) / sizeof(kTerms[0]); ++i) {
    NutationTerm term;
#ifdef ARDUINO
    memcpy_P(&term, &kTerms[i], sizeof(NutationTerm));
#else
    term = kTerms[i];
#endif
    float arg = term.nl * l + term.nlp * lp + term.nf * f + term.nd * d +
                term.nom * om;
    float sinArg = sinf(arg);
    float cosArg = cosf(arg);
    sumPsi += (term.sp + term.spt * t) * sinArg;
    sumEps += (term.ce + term.cet * t) * cosArg;
  }

  constexpr float kArcsecondToRad = kDegToRad / 3600.0f;
  deltaPsi = sumPsi * 1e-7f * kArcsecondToRad;
  deltaEpsilon = sumEps * 1e-7f * kArcsecondToRad;
}

Matrix3 precessionMatrixIAU2006(float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  float t5 = t4 * t;

  float eps0 = kObliquityJ2000;
  float psiA = (5038.481507f * t - 1.0790069f * t2 - 0.00114045f * t3 +
                0.000132851f * t4 - 9.51e-8f * t5) * (kDegToRad / 3600.0f);
  float omegaA = (84381.406f - 46.836769f * t - 0.0001831f * t2 +
                  0.00200340f * t3 - 5.76e-7f * t4 - 4.34e-8f * t5) *
                 (kDegToRad / 3600.0f);
  float chiA = (10.556403f * t - 2.3814292f * t2 - 0.00121197f * t3 +
                0.000170663f * t4 - 5.33e-8f * t5) * (kDegToRad / 3600.0f);

  Matrix3 r3chi = rotationZ(chiA);
  Matrix3 r1omega = rotationX(-omegaA);
  Matrix3 r3psi = rotationZ(-psiA);
  Matrix3 r1eps0 = rotationX(eps0);

  Matrix3 temp = multiply(r3psi, r1eps0);
  temp = multiply(r1omega, temp);
  return multiply(r3chi, temp);
}

Matrix3 nutationMatrix(float t, float deltaPsi, float deltaEpsilon) {
  float epsa = (84381.406f - 46.836769f * t - 0.0001831f * t * t +
                0.00200340f * t * t * t - 5.76e-7f * t * t * t * t -
                4.34e-8f * t * t * t * t * t) * (kDegToRad / 3600.0f);

  Matrix3 r1Minus = rotationX(-(epsa + deltaEpsilon));
  Matrix3 r3 = rotationZ(-deltaPsi);
  Matrix3 r1Plus = rotationX(epsa);
  return multiply(r1Minus, multiply(r3, r1Plus));
}

Vec3 applyPrecessionNutation(float jd, const Vec3& v) {
  float t = julianCenturies(jd);
  Matrix3 precession = precessionMatrixIAU2006(t);
  Vec3 precessed = transform(precession, v);

  float deltaPsi = 0.0f;
  float deltaEpsilon = 0.0f;
  computeNutation(t, deltaPsi, deltaEpsilon);
  Matrix3 nutation = nutationMatrix(t, deltaPsi, deltaEpsilon);
  return transform(nutation, precessed);
}

Vec3 aberrationCorrected(const Vec3& geo, const Vec3& observerVelocity) {
  float distance = magnitude(geo);
  if (distance == 0.0f) {
    return geo;
  }
  Vec3 direction = scale(geo, 1.0f / distance);
  Vec3 vOverC = scale(observerVelocity, 1.0f / kSpeedOfLightAuPerDay);
  float dotKV = dot(direction, vOverC);
  Vec3 adjusted = add(direction, subtract(vOverC, scale(direction, dotKV)));
  adjusted = unitVector(adjusted);
  return scale(adjusted, distance);
}

Vec3 earthVelocity(float jd) {
  float delta = 0.01f;
  Vec3 future = heliocentricPosition(PlanetId::Earth, jd + delta);
  Vec3 past = heliocentricPosition(PlanetId::Earth, jd - delta);
  Vec3 velocity = scale(subtract(future, past), 1.0f / (2.0f * delta));
  return velocity;
}

}  // namespace

namespace planets {

float julianDay(int year, int month, int day, float hourFraction) {
  if (month <= 2) {
    year -= 1;
    month += 12;
  }
  int A = year / 100;
  int B = 2 - A + A / 4;
  float jd = floorf(365.25f * (year + 4716)) + floorf(30.6001f * (month + 1)) +
             static_cast<float>(day) + static_cast<float>(B) - 1524.5f;
  jd += hourFraction / 24.0f;
  return jd;
}

bool computePlanet(PlanetId id, float julianDay, PlanetPosition& out) {
  if (id == PlanetId::Earth) {
    return false;
  }

  Vec3 earth = heliocentricPosition(PlanetId::Earth, julianDay);
  Vec3 planet = heliocentricPosition(id, julianDay);
  Vec3 geo = subtract(planet, earth);

  float distance = magnitude(geo);
  float lightTime = distance / kSpeedOfLightAuPerDay;
  Vec3 planetLight =
      heliocentricPosition(id, julianDay - lightTime);
  geo = subtract(planetLight, earth);
  distance = magnitude(geo);

  Vec3 velocity = earthVelocity(julianDay);
  geo = aberrationCorrected(geo, velocity);

  // Rotate from ecliptic to equatorial J2000.
  Matrix3 rot = rotationX(kObliquityJ2000);
  Vec3 equatorial = transform(rot, geo);

  // Apply precession and nutation to true equator of date.
  Vec3 apparent = applyPrecessionNutation(julianDay, equatorial);

  distance = magnitude(apparent);
  float ra = atan2f(apparent.y, apparent.x);
  if (ra < 0.0f) {
    ra += kTwoPi;
  }
  float dec = asinf(apparent.z / distance);

  out.raHours = normalizeRadians(ra) * kRadToHour;
  out.decDegrees = dec * kRadToDeg;
  out.distanceAu = distance;
  return true;
}

bool planetFromString(const String& name, PlanetId& id) {
  String lower = name;
  lower.toLowerCase();
  if (lower == "mercury") {
    id = PlanetId::Mercury;
    return true;
  }
  if (lower == "venus") {
    id = PlanetId::Venus;
    return true;
  }
  if (lower == "earth" || lower == "earth moon" || lower == "moon") {
    id = PlanetId::Earth;
    return false;
  }
  if (lower == "mars") {
    id = PlanetId::Mars;
    return true;
  }
  if (lower == "jupiter") {
    id = PlanetId::Jupiter;
    return true;
  }
  if (lower == "saturn") {
    id = PlanetId::Saturn;
    return true;
  }
  if (lower == "uranus") {
    id = PlanetId::Uranus;
    return true;
  }
  if (lower == "neptune") {
    id = PlanetId::Neptune;
    return true;
  }
  return false;
}

}  // namespace planets

