#pragma once

#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <frc/geometry/Rotation2d.h>

#include <units/angle.h>

namespace hb {

  /**
   * @brief Class for pigeon gyro inheriting the frc::Gyro class to ensure it interfaces withn the swerve drive
   */
  class PigeonGyro  {
  public:

    /**
     * @brief Creates a new pigeon gyro object using CAN 
     * 
     * @param ID the CAN ID
     */
    explicit PigeonGyro(int ID);

    /**
     * @brief Gets the angle the pigeon gyro is reading
     * 
     * @return angle in degrees 
     */
    double GetAngle() const;

    /**
     * @brief Gets the rate of rotation in degrees per second
     * 
     * @return double rate of rotation 
     */
    double GetRate() const;

    /**
     * @brief Resets the pigeon gyro heading
     */
    void Reset();

    /**
     * @brief Get the pitch of the gyro
     * 
     * @return double 
     */
    virtual double GetPitch() const;

    /**
     * @brief Get the roll of the gyro
     * 
     * @return double 
     */
    virtual double GetRoll() const;

    /**
     * Working version of GetRotation2d
    */
    virtual frc::Rotation2d GetRot2d() const;

    /**
     * @brief Get the rotation of the gyro in radians
     * 
     * @return units::radian_t 
     */
    virtual units::radian_t GetRad() const;

    /**
     * @brief Set the angle of the gyro
     * 
     * @param units::degree_t
     */
    virtual void SetPosition(units::degree_t);

    /**
     * @brief Get the compass heading of the gyro from [-180, 180]
     * 
     * @return double 
     */
    double GetCompassHeading() const;

    void Set(units::degree_t heading);

  private:
    ctre::phoenix::sensors::PigeonIMU* pigeon;
    mutable double m_angle;
    mutable double m_rate;
    double m_offset;
  };
} // namespace hb
