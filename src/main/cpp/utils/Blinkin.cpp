#include "utils/Blinkin.h"

using namespace hb;

Blinkin::Blinkin(uint8_t id) : m_blinkin(id) {}

void Blinkin::Set(double set) {
    m_blinkin.Set(set);
}
