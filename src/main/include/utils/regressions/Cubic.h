/**
 * @file Cubic.h
 * @author Nathan Correa
 * @date 2023-08-19
 */

#pragma once

#include "RegressionBase.h"

namespace hb {
  
  /**
   * The values entered in the constructor shoudld be the same ones coming from this desmos:
   * https://www.desmos.com/calculator/jjwe4f3fl0
   * 
   * The numbers should be entered into the contructor in the order found in desmos but for reference they will be this:
   * ax^3 + bx^2 + cx + d
   * 
   */
  class Cubic : public RegressionBase {

    public:

      /**
       * @brief Construct a new Cubic regression object
       * coefficients should be entered accoding to 
       * ax^3 + bx^2 + cx + d
       * 
       * @param a 
       * @param b 
       * @param c 
       * @param d 
       */
      Cubic(double a, double b, double c, double d);

      double Calculate(double x) const override;

    private:

      double m_a, m_b, m_c, m_d;
  };
} // namespace hb