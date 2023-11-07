// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Balance.h"

Balance::Balance(DriveSubsystem* drive) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void Balance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Balance::Execute() {
  double angle = m_drive->gyro.GetRoll();
  double calc = m_controller.Calculate(angle, 0.0);
  // printf("Gyro Calc: %5.2f\n", calc);
  if (fabs(angle) < 1.5) 
  m_drive->Drive(
    0_mps,
    0_mps,
    0_rad_per_s,
    true
  );
  else
  m_drive->Drive(
    units::meters_per_second_t(std::clamp(calc, -0.25, 0.25)),
    0_mps,
    units::radians_per_second_t(0),
    false
  );
}

// Called once the command ends or is interrupted.
void Balance::End(bool interrupted) {}

// Returns true when the command should end.
bool Balance::IsFinished() {
  return false;
}
