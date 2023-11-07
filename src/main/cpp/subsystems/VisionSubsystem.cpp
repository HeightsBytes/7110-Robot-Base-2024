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
m_leftEst(m_layout, photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(photonlib::PhotonCamera("Arducam_OV9281_USB_Camera_Left")), VisionConstants::LeftTransform),
m_xFilter(frc::LinearFilter<units::meter_t>::SinglePoleIIR(0.1, 0.02_s)),
m_yFilter(frc::LinearFilter<units::meter_t>::SinglePoleIIR(0.1, 0.02_s))
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

std::pair<std::optional<units::second_t>, std::optional<frc::Pose2d>> VisionSubsystem::GetPose() {
    std::optional<photonlib::EstimatedRobotPose> estl = m_leftEst.Update();
    std::optional<photonlib::EstimatedRobotPose> estr = m_rightEst.Update();
    std::pair<std::optional<frc::Pose2d>, std::optional<units::second_t>> estll = hb::LimeLight::GetPose();

    if (estll.first.has_value()) {
        return std::make_pair(estll.second, estll.first);
    } 

    bool filter = true;

    if (!estl.has_value() && !estr.has_value()) {
        return std::make_pair(std::nullopt, std::nullopt);
    } else if (estl.has_value() && !estr.has_value()) {
        frc::Pose2d pose = estl.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estl.value().timestamp;
        return std::make_pair(timestamp, pose);
    } else if (estr.has_value() && !estl.has_value()) {
        frc::Pose2d pose = estr.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estr.value().timestamp;
        return std::make_pair(timestamp, m_FilterPose(pose, filter));
    } 
    if (estl.value().targetsUsed.size() > estr.value().targetsUsed.size()) {
        frc::Pose2d pose = estl.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estl.value().timestamp;
        return std::make_pair(timestamp, m_FilterPose(pose, filter));
    } else if (estr.value().targetsUsed.size() > estl.value().targetsUsed.size()) {
        frc::Pose2d pose = estr.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estr.value().timestamp;
        return std::make_pair(timestamp, m_FilterPose(pose, filter));
    } else {
        frc::Pose2d pose = estl.value().estimatedPose.ToPose2d();
        units::second_t timestamp = estl.value().timestamp;
        return std::make_pair(timestamp, m_FilterPose(pose, filter));
    }
}


void VisionSubsystem::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Vision");

}

frc::Pose2d VisionSubsystem::m_FilterPose(frc::Pose2d pose, bool enabled) {
    units::meter_t x = m_xFilter.Calculate(pose.X());
    units::meter_t y = m_yFilter.Calculate(pose.Y());
    if (enabled)
    return frc::Pose2d(x, y, pose.Rotation());
    else 
    return pose;

}