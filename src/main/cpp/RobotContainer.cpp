// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// #pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "RobotContainer.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SelectCommand.h>
#include <frc2/command/Commands.h>


#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/Filesystem.h>
#include <frc/DriverStation.h>

#include <wpi/MemoryBuffer.h>

#include <utility>
#include <memory>
#include <stdexcept>

#include <units/angle.h>
#include <units/velocity.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/Balance.h"
#include "commands/ArmTo.h"
#include "commands/ClawFor.h"
#include "commands/DefaultDrive.h"

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClawSubsystem.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {

  m_chooser.AddOption("Test Auto", "test_auto");
  m_chooser.AddOption("Straight Line", "just_move");
  m_chooser.AddOption("Balance Path", "red_auto");
  m_chooser.AddOption("Crazy Auto", "red_crazy_auto");

  // Arm Commands
  NamedCommands::registerCommand("arm_mid_cone", std::make_shared<ArmTo>(&m_arm, ArmSubsystem::State::kMidCone));
  NamedCommands::registerCommand("arm_mid_cube", std::make_shared<ArmTo>(&m_arm, ArmSubsystem::State::kMidCubeConePickup));
  NamedCommands::registerCommand("arm_stow", std::make_shared<ArmTo>(&m_arm, ArmSubsystem::State::kStow));
  NamedCommands::registerCommand("arm_car", std::make_shared<ArmTo>(&m_arm, ArmSubsystem::State::kMsMaiCar));

  // Claw Commands
  NamedCommands::registerCommand("claw_open", std::make_shared<ClawFor>(&m_claw, ClawFor::Direction::kBackwards, 0.7_s));
  NamedCommands::registerCommand("claw_close", std::make_shared<ClawFor>(&m_claw, ClawFor::Direction::kForwards, 0.7_s));

  // Other Commands
  NamedCommands::registerCommand("drive_balance", std::make_shared<Balance>(&m_drive));
  NamedCommands::registerCommand("drive_switch", std::move(m_drive.SetGyro(180_deg)));

  frc::SmartDashboard::PutData("Arm", &m_arm);
  frc::SmartDashboard::PutData("Claw", &m_claw);
  frc::SmartDashboard::PutData("Swerve", &m_drive);
  frc::SmartDashboard::PutData("PDP", &m_pdp);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  m_drive.SetDefaultCommand(
    DefaultDrive(
      &m_drive,
      &m_driverController
    )
  );

}

void RobotContainer::ConfigureButtonBindings() {

  ConfigureDriverButtons();
  ConfigureOperatorButtons();

}

void RobotContainer::ConfigureDriverButtons() {

  m_driverController.A().OnTrue(ArmTo(&m_arm, ArmSubsystem::State::kMsMaiCar).ToPtr());
  
  m_driverController.Y().OnTrue(ArmTo(&m_arm, ArmSubsystem::State::kStow).ToPtr());

  m_driverController.RightBumper().OnTrue(m_claw.RunCMD(-0.7)).OnFalse(m_claw.RunCMD(0));

  m_driverController.LeftBumper().OnTrue(m_claw.RunCMD(0.7)).OnFalse(m_claw.RunCMD(0));

}

void RobotContainer::ConfigureOperatorButtons() {

}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

  std::string autoName = m_chooser.GetSelected();

	const std::string filePath = frc::filesystem::GetDeployDirectory()
			+ "/pathplanner/autos/" + autoName + ".auto";

	std::error_code error_code;
	std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
			wpi::MemoryBuffer::GetFile(filePath, error_code);

	if (fileBuffer == nullptr || error_code) {
		throw std::runtime_error("Cannot open file: " + filePath);
	}

	wpi::json json = wpi::json::parse(fileBuffer->GetCharBuffer());

	return AutoBuilder::getAutoCommandFromJson(json);

}
