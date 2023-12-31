#include "utils/regressions/Sinusoidal.h"

#include <cmath>

using namespace hb;

Sinusoidal::Sinusoidal(double a, double b, double c, double d) : m_a(a), m_b(b), m_c(c), m_d(d) {}

double Sinusoidal::Calculate(double x) const {
  return m_a * std::sin(m_b * (x - m_c)) + m_d;
}