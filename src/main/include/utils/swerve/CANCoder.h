#pragma once

#include <ctre/phoenix.h>

#include <units/angle.h>

#include <numbers>

namespace hb{
  /**
   * @brief CAN coder class for swerve modules
   * @warning ONLY FOR SWERVE MODULES
   */
  class S_CANCoder : CANCoder{
    public:
      explicit S_CANCoder(const int& id, const double& offset);

      /**
       * @brief gets the positon of the encoder from [-pi, pi]
       * and applies the offset
       * 
       * @return double
       */
      double Get();

    private:

      double m_offset;

      int m_ID;

    };
} // namespace hb
