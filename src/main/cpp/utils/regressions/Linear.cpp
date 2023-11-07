#include "utils/regressions/Linear.h"

using namespace hb;

Linear::Linear(double a, double b) : m_a(a), m_b(b) {}

double Linear::Calculate(double x) const {
  return m_a * x + m_b;
}