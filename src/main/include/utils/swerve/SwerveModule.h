// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/temperature.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "utils/swerve/CANCoder.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               const int turningEncoderPorts, const double offset);

  /**
   * Both the velocity of the swerve module and the angle of it
   * @returns frc::SwerveModuleState
   */
  frc::SwerveModuleState GetState() const;

  /**
   * Both the angle of the swerve module and the total distance of the drive
   * motor
   * @returns frc::SwerveModulePosition
   */
  frc::SwerveModulePosition GetPosition() const;

  /**
   * @param frc::SwerveModuleState the target state
   *
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  /**
   * Resets the drive and turn encoder
   * Turn encoder is zeroed with CANCoder
   * @warning also zeros drive position encoder which may throw off pose
   * estimation
   */
  void ResetEncoders();

  /**
   * Zeros the turn encoder with the CANCoder
   */
  void ZeroTurnEncoder();

  /**
   * Stops all outputs to the motors
   */
  void StopMotors();

 private:
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkMaxRelativeEncoder m_sparkDriveEncoder;
  rev::SparkMaxRelativeEncoder m_sparkTurnEncoder;

  rev::SparkMaxPIDController m_tController;
  rev::SparkMaxPIDController m_dController;

  hb::S_CANCoder m_turningEncoder;

  int m_id;
};
