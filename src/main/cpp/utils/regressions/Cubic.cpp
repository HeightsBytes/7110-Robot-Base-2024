#include "utils/regressions/Cubic.h"

#include <cmath>

using namespace hb;

Cubic::Cubic(double a, double b, double c, double d) : m_a(a), m_b(b), m_c(c), m_d(d) {}

double Cubic::Calculate(double x) const {
  return m_a * pow(x, 3) + m_b * pow(x, 2) + m_c * x + m_d;
}

