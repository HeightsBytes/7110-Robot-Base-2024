#include "utils/regressions/SquareRoot.h"

#include <cmath>

using namespace hb;

SquareRoot::SquareRoot(double a, double b, double h, double k) : m_a(a), m_b(b), m_h(h), m_k(k) {}

double SquareRoot::Calculate(double x) const {
  return m_a * sqrt(m_b * x - m_h) + m_k;
}