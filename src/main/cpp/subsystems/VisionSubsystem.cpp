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
#include "utils/cams/PosePacket.h"



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

std::vector<PosePacket> VisionSubsystem::GetPose() {

    std::vector<PosePacket> packets;


    try {
        std::optional<PosePacket> packet = hb::LimeLight::GetPose();
        if (packet.has_value()) {
            packets.emplace_back(packet.value());
        }
    } catch (...) {}

    try {
        std::optional<photonlib::EstimatedRobotPose> estl = m_leftEst.Update();
        if (estl.has_value()) {
            if (estl.value().strategy == photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR) {
                packets.emplace_back(PhotonToPosePacket(estl).value());
            }
        }
    } catch (...) {}
    
    try {
        std::optional<photonlib::EstimatedRobotPose> estr = m_rightEst.Update();
        if (estr.has_value()) {
            if (estr.value().strategy == photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR) {
                packets.emplace_back(PhotonToPosePacket(estr).value());
            }
        }
    } catch (...) {}

    return packets;
}


std::optional<PosePacket> VisionSubsystem::PhotonToPosePacket(std::optional<photonlib::EstimatedRobotPose> pose) {
    if (!pose.has_value()) return std::nullopt;
    
    return PosePacket(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
}
