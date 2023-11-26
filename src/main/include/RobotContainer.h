// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file RobotContainer.h
 * @date 2023-08-19
 */

#pragma once

#include <frc2/command/button/CommandXboxController.h>

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PowerDistribution.h>


//Subsystems
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClawSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "Constants.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:

  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:

  frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operatorController{OIConstants::kOperatorControllerPort};

  frc::PowerDistribution m_pdp{0, frc::PowerDistribution::ModuleType::kCTRE};

  // The robot's subsystems
  DriveSubsystem m_drive;
  ArmSubsystem m_arm;
  ClawSubsystem m_claw{&m_pdp};
  VisionSubsystem& m_vision = VisionSubsystem::GetInstance();

  // The chooser for the autonomous routines
  
  frc::SendableChooser<std::string> m_chooser;

  void ConfigureButtonBindings();

  void ConfigureDriverButtons();
  
  void ConfigureOperatorButtons();
};