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

    }; // class Blinkin

} // namespace hb