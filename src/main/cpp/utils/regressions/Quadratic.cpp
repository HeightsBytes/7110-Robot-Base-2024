#include "utils/regressions/Quadratic.h"

#include <cmath>

using namespace hb;

Quadratic::Quadratic(double a, double b, double c) : m_a(a), m_b(b), m_c(c) {}

double Quadratic::Calculate(double x) const {
  return m_a * pow(x, 2) + m_b * x + m_c;
}