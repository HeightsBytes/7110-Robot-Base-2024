// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultDriveCMD.h"

#include <utility>

#include "Constants.h"

using namespace units::velocity;
using namespace units::angular_velocity;

DefaultDriveCMD::DefaultDriveCMD(DriveSubsystem* subsystem, std::function<double()> x, 
  std::function<double()> y, std::function<double()> rot, std::function<bool()> field, std::function<double()> speed) :
  m_subsystem(subsystem), m_x(std::move(x)), m_y(std::move(y)), 
  m_rot(std::move(rot)), m_field(std::move(field)), m_speed(std::move(speed)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void DefaultDriveCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultDriveCMD::Execute() {
  // printf("x: %5.2f y: %5.2f rot %5.2f\n", m_x(), m_y(), m_rot());
  m_subsystem->Drive(
  meters_per_second_t(m_x() * m_speed()),
  meters_per_second_t(m_y() * m_speed()), 
  radians_per_second_t(m_rot()),
  true
  );
}

// Called once the command ends or is interrupted.
void DefaultDriveCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultDriveCMD::IsFinished() {
  return false;
}
