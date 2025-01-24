package us.ihmc.avatar.networkProcessor.modules.uiConnector;

import atlas_msgs.msg.dds.BDIBehaviorStatusPacket;
import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import perception_msgs.msg.dds.BlackFlyParameterPacket;
import perception_msgs.msg.dds.DetectedObjectPacket;
import perception_msgs.msg.dds.DrillDetectionPacket;
import perception_msgs.msg.dds.FisheyePacket;
import perception_msgs.msg.dds.HeatMapPacket;
import perception_msgs.msg.dds.HeightQuadTreeMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PointCloudWorldPacket;
import perception_msgs.msg.dds.ValveLocationPacket;
import perception_msgs.msg.dds.VideoPacket;
import toolbox_msgs.msg.dds.BehaviorControlModeResponsePacket;
import toolbox_msgs.msg.dds.BehaviorStatusPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;

import java.util.HashMap;

public class PacketsForwardedToTheUi
{
   public static final long UI_JOINT_CONFIGURATION_UPDATE_MILLIS = 100l;
   public static final long UI_WRIST_FEET_SENSORS_UPDATE_MILLIS = 500l;
   public static final long UI_MULTISENSE_IMU_CHECK_MILLIS=5000l;

   public final static Class<?>[] PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE = {
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
