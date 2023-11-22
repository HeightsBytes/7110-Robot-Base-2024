// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultDrive.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "utils/Util.h"
#include "Constants.h"

/**
#define X_OUT [this] {return -m_speedLimitx.Calculate(frc::ApplyDeadband(hb::sgn(m_driverController.GetLeftY()) * pow(m_driverController.GetLeftY(), 2), 0.01));}
#define Y_OUT [this] {return -m_speedLimity.Calculate(frc::ApplyDeadband(hb::sgn(m_driverController.GetLeftX()) * pow(m_driverController.GetLeftX(), 2), 0.01));}
#define ROT_OUT [this] {return -frc::ApplyDeadband(hb::sgn(m_driverController.GetRightX()) * pow(m_driverController.GetRightX(), 2), 0.025) * DriveConstants::kMaxAngularSpeed.value();}
*/

DefaultDrive::DefaultDrive(DriveSubsystem* drive, frc2::CommandXboxController* controller):
  m_drive(drive), m_controller(controller), m_maxSpeed(DriveConstants::kMaxSpeed.value()) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void DefaultDrive::Initialize() {
  frc::SmartDashboard::PutNumber("Max Speed", m_maxSpeed);
}

// Called repeatedly when this Command is scheduled to run
void DefaultDrive::Execute() {

  double maxSpeed = frc::SmartDashboard::GetNumber("Max Speed", DriveConstants::kMaxSpeed.value());
  maxSpeed = maxSpeed * (m_controller->GetRightTriggerAxis() * 0.625 + 0.375);

  // Note: x is forwards, y is side to side. 

  double x = -frc::ApplyDeadband(m_controller->GetLeftY(), 0.01);
  double y = frc::ApplyDeadband(m_controller->GetLeftX(), 0.01);
  double rot = -frc::ApplyDeadband(m_controller->GetRightX(), 0.025);


  double mag = std::pow(hb::hypot(x, y), 2) * maxSpeed;
  double angle = y == 0 ? hb::sgn(x)*std::numbers::pi/2 : std::atan(x/y);
  if (x > 0 && y < 0) angle += std::numbers::pi;
  if (x < 0 && y < 0) angle += std::numbers::pi;
  if (x == 0 && y < 0) angle += std::numbers::pi;

  units::meters_per_second_t xMove = units::meters_per_second_t(mag * std::sin(angle));
  units::meters_per_second_t yMove = -units::meters_per_second_t(mag * std::cos(angle));
  units::radians_per_second_t rotMove = units::radians_per_second_t(rot * DriveConstants::kMaxAngularSpeed.value());

  m_drive->Drive(
    xMove,
    yMove,
    rotMove,
    true
  );

}

// Called once the command ends or is interrupted.
void DefaultDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultDrive::IsFinished() {
  return false;
}
