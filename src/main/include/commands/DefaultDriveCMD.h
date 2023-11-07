// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file DeafaultDriveCMD.h
 * @date 2023-08-19
 */

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DefaultDriveCMD
    : public frc2::CommandHelper<frc2::Command, DefaultDriveCMD> {
 public:
  DefaultDriveCMD(DriveSubsystem*, std::function<double()>, std::function<double()>, std::function<double()>, std::function<bool()>,
  std::function<double()>);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_subsystem;
  std::function<double()> m_x;
  std::function<double()> m_y;
  std::function<double()> m_rot;
  std::function<bool()> m_field;
  std::function<double()> m_speed;
};
