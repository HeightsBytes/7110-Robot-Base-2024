// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file VisionSubsystem.h
 * @author Nathan Correa
 * @date 2023-08-19
 */

#pragma once

#include <optional>
#include <utility>

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/filter/LinearFilter.h>

#include <wpi/sendable/SendableBuilder.h>

#include <units/time.h>

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  static VisionSubsystem& GetInstance();

  photonlib::PhotonPipelineResult GetLeftFrame();

  photonlib::PhotonPipelineResult GetRightFrame();

  std::pair<std::optional<units::second_t>, std::optional<frc::Pose2d>> GetPose();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::Pose2d m_FilterPose(frc::Pose2d, bool);

  frc::AprilTagFieldLayout m_layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);


  photonlib::PhotonPoseEstimator m_rightEst;
  photonlib::PhotonPoseEstimator m_leftEst;

  frc::LinearFilter<units::meter_t> m_xFilter;
  frc::LinearFilter<units::meter_t> m_yFilter;

};
