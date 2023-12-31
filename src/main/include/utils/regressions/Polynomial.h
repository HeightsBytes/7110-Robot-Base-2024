#pragma once

#include <cmath>

#include <wpi/array.h>

#include "RegressionBase.h"

namespace hb {
    template<int terms>
    requires (terms > 0)
    class Polynomial : public RegressionBase {
        public:

            /** from least to greatest **/
            Polynomial(wpi::array<double, terms> constants) {
                m_constants = constants;
            }

            double Calculate(double input) const override {
                double output = 0;
                for (int i = 0; i < degree; i++) {
                    output += std::pow(input, i) * m_constants[i];
                }
            }

        private:

            wpi::array<double, terms> m_constants;

    };
} // namespace hb