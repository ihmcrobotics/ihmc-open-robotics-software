package us.ihmc.avatar.networkProcessor.modules.uiConnector;

import java.util.HashMap;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.BDIBehaviorStatusPacket;
import controller_msgs.msg.dds.BehaviorControlModeResponsePacket;
import controller_msgs.msg.dds.BehaviorStatusPacket;
import controller_msgs.msg.dds.BlackFlyParameterPacket;
import controller_msgs.msg.dds.BoundingBoxesPacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.DetectedObjectPacket;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.DrillDetectionPacket;
import controller_msgs.msg.dds.FisheyePacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.HeatMapPacket;
import controller_msgs.msg.dds.HeightQuadTreeMessage;
import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.LocalizationPointMapPacket;
import controller_msgs.msg.dds.ManipulationAbortedStatus;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.PointCloudWorldPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import controller_msgs.msg.dds.SnapFootstepPacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import controller_msgs.msg.dds.ValveLocationPacket;
import controller_msgs.msg.dds.VideoPacket;
import controller_msgs.msg.dds.WalkingStatusMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;

public class PacketsForwardedToTheUi
{
   public static final long UI_JOINT_CONFIGURATION_UPDATE_MILLIS = 100l;
   public static final long UI_WRIST_FEET_SENSORS_UPDATE_MILLIS = 500l;
   public static final long UI_MULTISENSE_IMU_CHECK_MILLIS=5000l;

   public static Class<?>[] PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE = {
      FootstepStatusMessage.class,
      PelvisPoseErrorPacket.class,
      BehaviorControlModeResponsePacket.class,
      BDIBehaviorStatusPacket.class,
      FootstepDataListMessage.class,
      PelvisHeightTrajectoryMessage.class,
      HeadTrajectoryMessage.class,
      NeckTrajectoryMessage.class,
      PelvisTrajectoryMessage.class,
      PelvisOrientationTrajectoryMessage.class,
      ChestTrajectoryMessage.class,
      SnapFootstepPacket.class,
      VideoPacket.class,
      HandTrajectoryMessage.class,
      ArmTrajectoryMessage.class,
      ValveLocationPacket.class,
      BehaviorStatusPacket.class,
      DoorLocationPacket.class,
      PointCloudWorldPacket.class,
      HandJointAnglePacket.class,
      WholeBodyTrajectoryMessage.class,
      ControllerCrashNotificationPacket.class,
      InvalidPacketNotificationPacket.class,
      DetectedObjectPacket.class,
      FisheyePacket.class,
      SimpleCoactiveBehaviorDataPacket.class,
      LocalizationPointMapPacket.class,
      BlackFlyParameterPacket.class,
      DrillDetectionPacket.class,
      ManipulationAbortedStatus.class,
      KinematicsToolboxOutputStatus.class,
      WholeBodyTrajectoryToolboxOutputStatus.class,
      FootstepPlanningToolboxOutputStatus.class,
      TextToSpeechPacket.class,
      UIPositionCheckerPacket.class,
      PlanarRegionsListMessage.class,
      HeightQuadTreeMessage.class,
      LidarScanMessage.class,
      HeatMapPacket.class,
      BoundingBoxesPacket.class,
      WalkingStatusMessage.class,
      StereoVisionPointCloudMessage.class
   };

   public static final HashMap<Class<?>, Long> PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS = new HashMap<Class<?>, Long>();
   static {
//      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(RobotPoseData.class, UI_JOINT_CONFIGURATION_UPDATE_MILLIS);
      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(CapturabilityBasedStatus.class, UI_JOINT_CONFIGURATION_UPDATE_MILLIS);
      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(RobotConfigurationData.class, UI_JOINT_CONFIGURATION_UPDATE_MILLIS);
   }


}
