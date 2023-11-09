// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <units/length.h>
#include <units/angle.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "Constants.h"
#include "utils/cams/Limelight.h"

VisionSubsystem::VisionSubsystem() : 
m_rightEst(m_layout, photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(photonlib::PhotonCamera("Arducam_OV9281_USB_Camera_Right")), VisionConstants::RightTransform),
m_leftEst(m_layout, photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(photonlib::PhotonCamera("Arducam_OV9281_USB_Camera_Left")), VisionConstants::LeftTransform)
{
    m_leftEst.SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);
    m_rightEst.SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {}

VisionSubsystem& VisionSubsystem::GetInstance() {
    static VisionSubsystem inst;
    return inst;
}

photonlib::PhotonPipelineResult VisionSubsystem::GetLeftFrame() {
    return m_leftEst.GetCamera()->GetLatestResult();
}

photonlib::PhotonPipelineResult VisionSubsystem::GetRightFrame() {
    return m_rightEst.GetCamera()->GetLatestResult();
}

PosePacket_t VisionSubsystem::GetPose() {
    std::optional<photonlib::EstimatedRobotPose> estl = m_leftEst.Update();
    std::optional<photonlib::EstimatedRobotPose> estr = m_rightEst.Update();
    PosePacket_t estll = hb::LimeLight::GetPose();

    // Limelight is most accurate, take values here first
    if (estll.has_value()) {
        return estll;
    } 

    
    // Check to see if photoncameras do not have a value
    if (!estl.has_value() && !estr.has_value()) {

        return std::nullopt;

    }

    // Check to see if one camera has an estimate
    if (estl.has_value() && !estr.has_value()) {

        frc::Pose2d pose = estl.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estl.value().timestamp;

        return std::make_pair(timestamp, pose);

    } else if (estr.has_value() && !estl.has_value()) {

        frc::Pose2d pose = estr.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estr.value().timestamp;

        return std::make_pair(timestamp, pose);

    } 

    // Check to see which cam sees the most targets
    if (estl.value().targetsUsed.size() > estr.value().targetsUsed.size()) {
        frc::Pose2d pose = estl.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estl.value().timestamp;
        return std::make_pair(timestamp, pose);
    } else if (estr.value().targetsUsed.size() > estl.value().targetsUsed.size()) {
        frc::Pose2d pose = estr.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estr.value().timestamp;
        return std::make_pair(timestamp, pose);
    } else {
        frc::Pose2d pose = estl.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estl.value().timestamp;
        return std::make_pair(timestamp, pose);
    }
}


void VisionSubsystem::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Vision");

}
