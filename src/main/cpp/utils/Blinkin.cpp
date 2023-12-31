// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/Blinkin.h"

using namespace hb;

Blinkin::Blinkin(uint8_t id) : m_blinkin(id) {}

void Blinkin::Set(double set) {
  m_blinkin.Set(set);
}
