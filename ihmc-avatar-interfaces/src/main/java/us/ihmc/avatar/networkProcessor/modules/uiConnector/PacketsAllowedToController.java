package us.ihmc.avatar.networkProcessor.modules.uiConnector;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.AdjustFootstepMessage;
import controller_msgs.msg.dds.ArmDesiredAccelerationsMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.AtlasDesiredPumpPSIPacket;
import controller_msgs.msg.dds.AtlasElectricMotorAutoEnableFlagPacket;
import controller_msgs.msg.dds.AtlasElectricMotorEnablePacket;
import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket;
import controller_msgs.msg.dds.AutomaticManipulationAbortMessage;
import controller_msgs.msg.dds.BDIBehaviorCommandPacket;
import controller_msgs.msg.dds.CenterOfMassTrajectoryMessage;
import controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.ClearDelayQueueMessage;
import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.FootLoadBearingMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.IMUPacket;
import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import controller_msgs.msg.dds.LegCompliancePacket;
import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.LocalizationPointMapPacket;
import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.MomentumTrajectoryMessage;
import controller_msgs.msg.dds.NeckDesiredAccelerationsMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessage;
import controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket;
import controller_msgs.msg.dds.SpineDesiredAccelerationsMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StateEstimatorModePacket;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;

public class PacketsAllowedToController
{
   public final static Class<?>[] PACKETS_ALLOWED_TO_CONTROLLER = {
         MessageCollection.class,
         IMUPacket.class,
         AutomaticManipulationAbortMessage.class,
         FootLoadBearingMessage.class,
         HandLoadBearingMessage.class,
         ArmDesiredAccelerationsMessage.class,
         NeckDesiredAccelerationsMessage.class,
         HandTrajectoryMessage.class,
         ArmTrajectoryMessage.class,
         HeadTrajectoryMessage.class,
         NeckTrajectoryMessage.class,
         ChestTrajectoryMessage.class,
         SpineTrajectoryMessage.class,
         SpineDesiredAccelerationsMessage.class,
         PelvisTrajectoryMessage.class,
         PelvisOrientationTrajectoryMessage.class,
         FootTrajectoryMessage.class,
         WholeBodyTrajectoryMessage.class,
         PelvisHeightTrajectoryMessage.class,
         StopAllTrajectoryMessage.class,
         GoHomeMessage.class,
         HandHybridJointspaceTaskspaceTrajectoryMessage.class,
         ChestHybridJointspaceTaskspaceTrajectoryMessage.class,
         HeadHybridJointspaceTaskspaceTrajectoryMessage.class,
         ClearDelayQueueMessage.class,
         MomentumTrajectoryMessage.class,
         CenterOfMassTrajectoryMessage.class,
         FootstepDataMessage.class,
         AdjustFootstepMessage.class,
         FootstepDataListMessage.class,
         PauseWalkingMessage.class,
         AbortWalkingMessage.class,
         PrepareForLocomotionMessage.class,
         PlanarRegionsListMessage.class,
         PlanarRegionMessage.class,
         RequestPlanarRegionsListMessage.class,
         StampedPosePacket.class,
         HighLevelStateMessage.class,
         BDIBehaviorCommandPacket.class,
         LocalizationPacket.class,
         PelvisPoseErrorPacket.class,
         LocalizationPointMapPacket.class,
         ControllerCrashNotificationPacket.class,
         InvalidPacketNotificationPacket.class,
         AtlasLowLevelControlModeMessage.class,
         AtlasWristSensorCalibrationRequestPacket.class,
         AtlasElectricMotorEnablePacket.class,
         AtlasElectricMotorAutoEnableFlagPacket.class,
         AtlasDesiredPumpPSIPacket.class,
         StateEstimatorModePacket.class,
         RequestWristForceSensorCalibrationPacket.class,
         LegCompliancePacket.class,
         TextToSpeechPacket.class
         };
}
