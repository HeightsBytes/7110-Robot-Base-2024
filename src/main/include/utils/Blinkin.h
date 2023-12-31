// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/Spark.h>
#include <stdint.h>

namespace hb {

class Blinkin {
 public:
  /**
   * @brief Construct a new Blinkin object
   *
   * @param id pwm
   */
  explicit Blinkin(uint8_t id);

  /**
   * @brief Sets the pattern of the blinkin
   *
   * @param double [-1,1]
   */
  void Set(double set);

 private:
  frc::Spark m_blinkin;

};  // class Blinkin

}  // namespace hb
