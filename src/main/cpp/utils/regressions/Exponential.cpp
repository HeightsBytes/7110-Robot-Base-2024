#include "utils/regressions/Exponential.h"

#include <cmath>

using namespace hb;

Exponential::Exponential(double a, double b) : m_a(a), m_b(b) {}

double Exponential::Calculate(double x) const {
  return m_a * pow(m_b, x);
}