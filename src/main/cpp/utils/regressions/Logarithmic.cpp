#include "utils/regressions/Logarithmic.h"

#include <cmath>

using namespace hb;

Logarithmic::Logarithmic(double a, double b) : m_a(a), m_b(b) {}

double Logarithmic::Calculate(double x) const {
  return m_a + m_b * std::log(x); // std::log is natural log
}