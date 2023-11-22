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
#include <vector>

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

typedef std::optional<std::pair<units::second_t, frc::Pose2d>> PosePacket_t;

/**
 * 
 * To update the Orange Pi:
 * 1. Download the latest photonvision .jar file
 * 2. Go into bash (windows subsystem for linux required)
 * 3. sch [photonvision jar].jar orangepi@[module name]:~/
 * 4. ssh orangepi@[module name]
 * 5. sudo mv [photonvision jar].jar /opt/photonvision/photonvision.jar
 * 6. sudo systemctl restart photonvision.service
 * 7. sudo reboot now
 * 
*/
class VisionSubsystem : public frc2::SubsystemBase {
 public:

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  static VisionSubsystem& GetInstance();

  photonlib::PhotonPipelineResult GetLeftFrame();

  photonlib::PhotonPipelineResult GetRightFrame();

  std::vector<PosePacket_t> GetPose();

 private:

  VisionSubsystem();

  PosePacket_t PhotonToPosePacket(std::optional<photonlib::EstimatedRobotPose> pose);

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::AprilTagFieldLayout m_layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);

  photonlib::PhotonPoseEstimator m_rightEst;
  photonlib::PhotonPoseEstimator m_leftEst;

};
