/**
 * @file Quadratic.h
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
   * ax^2 + bx + c
   * 
   */
  class Quadratic : public RegressionBase { 

    public:

      /**
       * @brief Construct a new Quadratic regression object
       * the coeffients should be entered according to 
       * ax^2 + bx + c
       * 
       * @param a 
       * @param b 
       * @param c 
       */
      explicit Quadratic(double a, double b, double c);

      double Calculate(double x) const override;

    private:

      double m_a, m_b, m_c;

  };
} // namespace hb