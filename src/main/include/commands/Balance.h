// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file Balance.h
 * @date 2023-08-19
 */

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/PIDController.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Balance
    : public frc2::CommandHelper<frc2::Command, Balance> {
 public:
  Balance(DriveSubsystem* drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

    DriveSubsystem* m_drive;

    frc::PIDController m_controller{0.03, 0, 0.007};


};
