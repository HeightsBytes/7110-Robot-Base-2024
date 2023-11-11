// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DriverStation.h>
#include <frc/filter/MedianFilter.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <wpi/sendable/SendableBuilder.h>

#include <cmath>
#include <ranges>

#include "utils/cams/Limelight.h"
#include "Constants.h"
#include "utils/Util.h"

using namespace DriveConstants;
using namespace DriveConstants::CanIds;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
    : 
      m_frontLeft{
          kFrontLeftDriveMotorPort,       kFrontLeftTurningMotorPort,
          kFrontLeftTurningEncoderPorts,  kFrontLeftOffset},

      m_rearLeft{
          kRearLeftDriveMotorPort,        kRearLeftTurningMotorPort,
          kRearLeftTurningEncoderPorts,   kRearLeftOffset},

      m_frontRight{
          kFrontRightDriveMotorPort,      kFrontRightTurningMotorPort,
          kFrontRightTurningEncoderPorts, kFrontRightOffset},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightTurningEncoderPorts,  kRearRightOffset},
      m_visionSystem(VisionSubsystem::GetInstance()), 
      m_poseEstimator(kDriveKinematics, gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()),
                    m_vision(true), 
                    m_target(Target::kCone)
                     {
                      frc::SmartDashboard::PutData("Field", &m_field);
                       AutoBuilder::configureHolonomic(
                          [this] {return GetPose();},
                          [this](frc::Pose2d pose) {SetPose(pose);},
                          [this] {return GetVelocity();},
                          [this](frc::ChassisSpeeds speeds) {DriveRobotRelative(speeds, true);},
                          AutoConstants::kConfig, 
                          this
                       );
                    }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_poseEstimator.Update(gyro.GetRot2d(), GetModulePositions());


  if (m_vision) {
    std::vector<PosePacket_t> CamPose = m_visionSystem.GetPose();
    for (PosePacket_t i : CamPose | std::views::filter([](PosePacket_t packet) {return packet.has_value();})) {
      m_poseEstimator.AddVisionMeasurement(i.value().second, i.value().first);
    }
  }

  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, gyro.GetRot2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

}

void DriveSubsystem::DriveFieldRelative(frc::ChassisSpeeds speeds) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, gyro.GetRot2d()));

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  frc::SmartDashboard::PutNumber("Angle", fl.angle.Degrees().value());

  // printf("speed: %5.2f\t angle: %5.2f\n", fr.speed.value(), fr.angle.Degrees().value());

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br); 
}

void DriveSubsystem::DriveRobotRelative(frc::ChassisSpeeds speeds, bool auton) {
  auto states = kDriveKinematics.ToSwerveModuleStates(speeds);
  kDriveKinematics.DesaturateWheelSpeeds(&states, auton ? AutoConstants::kMaxSpeed : DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br); 
}

frc::ChassisSpeeds DriveSubsystem::GetVelocity() {
  return kDriveKinematics.ToChassisSpeeds(GetModuleStates());
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

wpi::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() {
  return {
    m_frontLeft.GetState(),
    m_frontRight.GetState(),
    m_rearLeft.GetState(),
    m_rearRight.GetState()
  };
}

wpi::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {
    m_frontLeft.GetPosition(),
    m_frontRight.GetPosition(),
    m_rearLeft.GetPosition(),
    m_rearRight.GetPosition()
  };
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::SetPose(frc::Pose2d pose) {
  m_poseEstimator.ResetPosition(gyro.GetRot2d(), GetModulePositions(), pose);
}


void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ZeroTurnEncoder();
  m_frontRight.ZeroTurnEncoder();
  m_rearLeft.ZeroTurnEncoder();
  m_rearRight.ZeroTurnEncoder();
}

void DriveSubsystem::SetTarget(DriveSubsystem::Target target) {
  m_target = target;
}

DriveSubsystem::Target DriveSubsystem::GetTarget() {
  return m_target;
}

static double hyp(double x, double y) {
  return sqrt(x*x + y*y);
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Swerve Drive");

  builder.AddBooleanProperty("Vision", LAMBDA(m_vision), [this](bool set) -> void {m_vision = set;});

  builder.AddDoubleProperty("X Velocity", LAMBDA(GetVelocity().vx.value()), nullptr);
  builder.AddDoubleProperty("Y Velocity", LAMBDA(GetVelocity().vy.value()), nullptr);
  builder.AddDoubleProperty("Rotation Velocity", LAMBDA(GetVelocity().omega.value()), nullptr);
  builder.AddDoubleProperty("Mag Velocity", LAMBDA(hyp(GetVelocity().vx.value(), GetVelocity().vy.value())), nullptr);


}
