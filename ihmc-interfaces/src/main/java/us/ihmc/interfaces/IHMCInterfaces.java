package us.ihmc.interfaces;

import controller_msgs.msg.dds.*;
import geometry_msgs.msg.dds.Vector3PubSubType;
import std_msgs.msg.dds.Int32;
import std_msgs.msg.dds.Int32PubSubType;
import std_msgs.msg.dds.Int64;
import std_msgs.msg.dds.Int64PubSubType;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pubsub.TopicDataType;

import java.util.HashMap;

public class IHMCInterfaces
{
   public static final HashMap<Class<?>, Class<? extends TopicDataType>> PUB_SUB_TYPES = new HashMap<>();

   private static boolean mapHasBeenFilled = false;

   public static boolean contains(Class<?> messageClazz)
   {
      if (!mapHasBeenFilled)
         fillMap();

      boolean contains = PUB_SUB_TYPES.containsKey(messageClazz);

      if (!contains)
      {
         PrintTools.warn(IHMCInterfaces.class,
                         "Class not ROS2 compatible. Please add PUB_SUB_TYPES.put(" + messageClazz.getSimpleName() + ".class, " + messageClazz
                               .getSimpleName() + "PubSubType.class);");
      }

      return contains;
   }

   public static TopicDataType getPubSubType(Class<?> messageClazz)
   {
      try
      {
         return PUB_SUB_TYPES.get(messageClazz).newInstance();
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public static void fillMap()
   {
      PUB_SUB_TYPES.put(Vector3D.class, Vector3PubSubType.class);
      PUB_SUB_TYPES.put(Int32.class, Int32PubSubType.class);
      PUB_SUB_TYPES.put(Int64.class, Int64PubSubType.class);
      PUB_SUB_TYPES.put(MessageCollection.class, MessageCollectionPubSubType.class);
      PUB_SUB_TYPES.put(MessageCollectionNotification.class, MessageCollectionNotificationPubSubType.class);
      PUB_SUB_TYPES.put(HandCollisionDetectedPacket.class, HandCollisionDetectedPacketPubSubType.class);
      PUB_SUB_TYPES.put(VideoPacket.class, VideoPacketPubSubType.class);
      PUB_SUB_TYPES.put(SimulatedLidarScanPacket.class, SimulatedLidarScanPacketPubSubType.class);
      PUB_SUB_TYPES.put(IMUPacket.class, IMUPacketPubSubType.class);
      PUB_SUB_TYPES.put(AutomaticManipulationAbortMessage.class, AutomaticManipulationAbortMessagePubSubType.class);
      PUB_SUB_TYPES.put(ManipulationAbortedStatus.class, ManipulationAbortedStatusPubSubType.class);
      PUB_SUB_TYPES.put(HandDesiredConfigurationMessage.class, HandDesiredConfigurationMessagePubSubType.class);
      PUB_SUB_TYPES.put(ObjectWeightPacket.class, ObjectWeightPacketPubSubType.class);
      PUB_SUB_TYPES.put(FootLoadBearingMessage.class, FootLoadBearingMessagePubSubType.class);
      PUB_SUB_TYPES.put(HandLoadBearingMessage.class, HandLoadBearingMessagePubSubType.class);
      PUB_SUB_TYPES.put(ArmDesiredAccelerationsMessage.class, ArmDesiredAccelerationsMessagePubSubType.class);
      PUB_SUB_TYPES.put(NeckDesiredAccelerationsMessage.class, NeckDesiredAccelerationsMessagePubSubType.class);
      PUB_SUB_TYPES.put(HandTrajectoryMessage.class, HandTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(ArmTrajectoryMessage.class, ArmTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(HeadTrajectoryMessage.class, HeadTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(NeckTrajectoryMessage.class, NeckTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(ChestTrajectoryMessage.class, ChestTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(SpineTrajectoryMessage.class, SpineTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(SpineDesiredAccelerationsMessage.class, SpineDesiredAccelerationsMessagePubSubType.class);
      PUB_SUB_TYPES.put(PelvisTrajectoryMessage.class, PelvisTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(PelvisOrientationTrajectoryMessage.class, PelvisOrientationTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(FootTrajectoryMessage.class, FootTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(WholeBodyTrajectoryMessage.class, WholeBodyTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(PelvisHeightTrajectoryMessage.class, PelvisHeightTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(StopAllTrajectoryMessage.class, StopAllTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(GoHomeMessage.class, GoHomeMessagePubSubType.class);
      PUB_SUB_TYPES.put(HandHybridJointspaceTaskspaceTrajectoryMessage.class, HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(ChestHybridJointspaceTaskspaceTrajectoryMessage.class, ChestHybridJointspaceTaskspaceTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(HeadHybridJointspaceTaskspaceTrajectoryMessage.class, HeadHybridJointspaceTaskspaceTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(ClearDelayQueueMessage.class, ClearDelayQueueMessagePubSubType.class);
      PUB_SUB_TYPES.put(MomentumTrajectoryMessage.class, MomentumTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(CenterOfMassTrajectoryMessage.class, CenterOfMassTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(OneDoFJointTrajectoryMessage.class, OneDoFJointTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(TrajectoryPoint1DMessage.class, TrajectoryPoint1DMessagePubSubType.class);
      PUB_SUB_TYPES.put(EuclideanTrajectoryPointMessage.class, EuclideanTrajectoryPointMessagePubSubType.class);
      PUB_SUB_TYPES.put(SO3TrajectoryPointMessage.class, SO3TrajectoryPointMessagePubSubType.class);
      PUB_SUB_TYPES.put(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessagePubSubType.class);
      PUB_SUB_TYPES.put(WalkingControllerFailureStatusMessage.class, WalkingControllerFailureStatusMessagePubSubType.class);
      PUB_SUB_TYPES.put(VehiclePosePacket.class, VehiclePosePacketPubSubType.class);
      PUB_SUB_TYPES.put(ToolboxStateMessage.class, ToolboxStateMessagePubSubType.class);
      PUB_SUB_TYPES.put(FootstepPlanningToolboxOutputStatus.class, FootstepPlanningToolboxOutputStatusPubSubType.class);
      PUB_SUB_TYPES.put(FootstepPlanningRequestPacket.class, FootstepPlanningRequestPacketPubSubType.class);
      PUB_SUB_TYPES.put(KinematicsToolboxOutputStatus.class, KinematicsToolboxOutputStatusPubSubType.class);
      PUB_SUB_TYPES.put(KinematicsToolboxCenterOfMassMessage.class, KinematicsToolboxCenterOfMassMessagePubSubType.class);
      PUB_SUB_TYPES.put(KinematicsToolboxRigidBodyMessage.class, KinematicsToolboxRigidBodyMessagePubSubType.class);
      PUB_SUB_TYPES.put(KinematicsToolboxConfigurationMessage.class, KinematicsToolboxConfigurationMessagePubSubType.class);
      PUB_SUB_TYPES.put(HumanoidKinematicsToolboxConfigurationMessage.class, HumanoidKinematicsToolboxConfigurationMessagePubSubType.class);
      PUB_SUB_TYPES.put(WholeBodyTrajectoryToolboxMessage.class, WholeBodyTrajectoryToolboxMessagePubSubType.class);
      PUB_SUB_TYPES.put(WholeBodyTrajectoryToolboxConfigurationMessage.class, WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.class);
      PUB_SUB_TYPES.put(WaypointBasedTrajectoryMessage.class, WaypointBasedTrajectoryMessagePubSubType.class);
      PUB_SUB_TYPES.put(ReachingManifoldMessage.class, ReachingManifoldMessagePubSubType.class);
      PUB_SUB_TYPES.put(RigidBodyExplorationConfigurationMessage.class, RigidBodyExplorationConfigurationMessagePubSubType.class);
      PUB_SUB_TYPES.put(RobotConfigurationData.class, RobotConfigurationDataPubSubType.class);
      PUB_SUB_TYPES.put(AtlasAuxiliaryRobotData.class, AtlasAuxiliaryRobotDataPubSubType.class);
      PUB_SUB_TYPES.put(FootstepDataMessage.class, FootstepDataMessagePubSubType.class);
      PUB_SUB_TYPES.put(AdjustFootstepMessage.class, AdjustFootstepMessagePubSubType.class);
      PUB_SUB_TYPES.put(FootstepDataListMessage.class, FootstepDataListMessagePubSubType.class);
      PUB_SUB_TYPES.put(PauseWalkingMessage.class, PauseWalkingMessagePubSubType.class);
      PUB_SUB_TYPES.put(FootstepStatusMessage.class, FootstepStatusMessagePubSubType.class);
      PUB_SUB_TYPES.put(PlanOffsetStatus.class, PlanOffsetStatusPubSubType.class);
      PUB_SUB_TYPES.put(WalkingStatusMessage.class, WalkingStatusMessagePubSubType.class);
      PUB_SUB_TYPES.put(AbortWalkingMessage.class, AbortWalkingMessagePubSubType.class);
      PUB_SUB_TYPES.put(PrepareForLocomotionMessage.class, PrepareForLocomotionMessagePubSubType.class);
      PUB_SUB_TYPES.put(PlanarRegionsListMessage.class, PlanarRegionsListMessagePubSubType.class);
      PUB_SUB_TYPES.put(PlanarRegionMessage.class, PlanarRegionMessagePubSubType.class);
      PUB_SUB_TYPES.put(RequestPlanarRegionsListMessage.class, RequestPlanarRegionsListMessagePubSubType.class);
      PUB_SUB_TYPES.put(SCSListenerPacket.class, SCSListenerPacketPubSubType.class);
      PUB_SUB_TYPES.put(StampedPosePacket.class, StampedPosePacketPubSubType.class);
      PUB_SUB_TYPES.put(DetectedObjectPacket.class, DetectedObjectPacketPubSubType.class);
      PUB_SUB_TYPES.put(HighLevelStateMessage.class, HighLevelStateMessagePubSubType.class);
      PUB_SUB_TYPES.put(HighLevelStateChangeStatusMessage.class, HighLevelStateChangeStatusMessagePubSubType.class);
      PUB_SUB_TYPES.put(ManualHandControlPacket.class, ManualHandControlPacketPubSubType.class);
      PUB_SUB_TYPES.put(HandPowerCyclePacket.class, HandPowerCyclePacketPubSubType.class);
      PUB_SUB_TYPES.put(HandJointAnglePacket.class, HandJointAnglePacketPubSubType.class);
      PUB_SUB_TYPES.put(BDIBehaviorCommandPacket.class, BDIBehaviorCommandPacketPubSubType.class);
      PUB_SUB_TYPES.put(BDIBehaviorStatusPacket.class, BDIBehaviorStatusPacketPubSubType.class);
      PUB_SUB_TYPES.put(FisheyePacket.class, FisheyePacketPubSubType.class);
      PUB_SUB_TYPES.put(MultisenseParameterPacket.class, MultisenseParameterPacketPubSubType.class);
      PUB_SUB_TYPES.put(HumanoidBehaviorTypePacket.class, HumanoidBehaviorTypePacketPubSubType.class);
      PUB_SUB_TYPES.put(BehaviorControlModePacket.class, BehaviorControlModePacketPubSubType.class);
      PUB_SUB_TYPES.put(BehaviorControlModeResponsePacket.class, BehaviorControlModeResponsePacketPubSubType.class);
      PUB_SUB_TYPES.put(HandDesiredConfigurationMessage.class, HandDesiredConfigurationMessagePubSubType.class);
      PUB_SUB_TYPES.put(ManualHandControlPacket.class, ManualHandControlPacketPubSubType.class);
      PUB_SUB_TYPES.put(MultisenseParameterPacket.class, MultisenseParameterPacketPubSubType.class);
      PUB_SUB_TYPES.put(SnapFootstepPacket.class, SnapFootstepPacketPubSubType.class);
      PUB_SUB_TYPES.put(BlackFlyParameterPacket.class, BlackFlyParameterPacketPubSubType.class);
      PUB_SUB_TYPES.put(WalkToGoalBehaviorPacket.class, WalkToGoalBehaviorPacketPubSubType.class);
      PUB_SUB_TYPES.put(FootstepPlanRequestPacket.class, FootstepPlanRequestPacketPubSubType.class);
      PUB_SUB_TYPES.put(SimpleCoactiveBehaviorDataPacket.class, SimpleCoactiveBehaviorDataPacketPubSubType.class);
      PUB_SUB_TYPES.put(WallPosePacket.class, WallPosePacketPubSubType.class);
      PUB_SUB_TYPES.put(WalkOverTerrainGoalPacket.class, WalkOverTerrainGoalPacketPubSubType.class);
      PUB_SUB_TYPES.put(DoorLocationPacket.class, DoorLocationPacketPubSubType.class);
      PUB_SUB_TYPES.put(ValveLocationPacket.class, ValveLocationPacketPubSubType.class);
      PUB_SUB_TYPES.put(BehaviorStatusPacket.class, BehaviorStatusPacketPubSubType.class);
      PUB_SUB_TYPES.put(CapturabilityBasedStatus.class, CapturabilityBasedStatusPubSubType.class);
      PUB_SUB_TYPES.put(FootstepPathPlanPacket.class, FootstepPathPlanPacketPubSubType.class);
      PUB_SUB_TYPES.put(LocalizationPacket.class, LocalizationPacketPubSubType.class);
      PUB_SUB_TYPES.put(LocalizationStatusPacket.class, LocalizationStatusPacketPubSubType.class);
      PUB_SUB_TYPES.put(PelvisPoseErrorPacket.class, PelvisPoseErrorPacketPubSubType.class);
      PUB_SUB_TYPES.put(LocalizationPointMapPacket.class, LocalizationPointMapPacketPubSubType.class);
      PUB_SUB_TYPES.put(PointCloudWorldPacket.class, PointCloudWorldPacketPubSubType.class);
      PUB_SUB_TYPES.put(HeightQuadTreeToolboxRequestMessage.class, HeightQuadTreeToolboxRequestMessagePubSubType.class);
      PUB_SUB_TYPES.put(HeightQuadTreeMessage.class, HeightQuadTreeMessagePubSubType.class);
      PUB_SUB_TYPES.put(LidarScanMessage.class, LidarScanMessagePubSubType.class);
      PUB_SUB_TYPES.put(RequestLidarScanMessage.class, RequestLidarScanMessagePubSubType.class);
      PUB_SUB_TYPES.put(ControllerCrashNotificationPacket.class, ControllerCrashNotificationPacketPubSubType.class);
      PUB_SUB_TYPES.put(InvalidPacketNotificationPacket.class, InvalidPacketNotificationPacketPubSubType.class);
      PUB_SUB_TYPES.put(AtlasLowLevelControlModeMessage.class, AtlasLowLevelControlModeMessagePubSubType.class);
      PUB_SUB_TYPES.put(AtlasWristSensorCalibrationRequestPacket.class, AtlasWristSensorCalibrationRequestPacketPubSubType.class);
      PUB_SUB_TYPES.put(AtlasElectricMotorEnablePacket.class, AtlasElectricMotorEnablePacketPubSubType.class);
      PUB_SUB_TYPES.put(AtlasElectricMotorAutoEnableFlagPacket.class, AtlasElectricMotorAutoEnableFlagPacketPubSubType.class);
      PUB_SUB_TYPES.put(AtlasDesiredPumpPSIPacket.class, AtlasDesiredPumpPSIPacketPubSubType.class);
      PUB_SUB_TYPES.put(StateEstimatorModePacket.class, StateEstimatorModePacketPubSubType.class);
      PUB_SUB_TYPES.put(RequestWristForceSensorCalibrationPacket.class, RequestWristForceSensorCalibrationPacketPubSubType.class);
      PUB_SUB_TYPES.put(UIConnectedPacket.class, UIConnectedPacketPubSubType.class);
      PUB_SUB_TYPES.put(LegCompliancePacket.class, LegCompliancePacketPubSubType.class);
      PUB_SUB_TYPES.put(DrillDetectionPacket.class, DrillDetectionPacketPubSubType.class);
      PUB_SUB_TYPES.put(UIPositionCheckerPacket.class, UIPositionCheckerPacketPubSubType.class);
      PUB_SUB_TYPES.put(TextToSpeechPacket.class, TextToSpeechPacketPubSubType.class);
      PUB_SUB_TYPES.put(HeatMapPacket.class, HeatMapPacketPubSubType.class);
      PUB_SUB_TYPES.put(BoundingBoxesPacket.class, BoundingBoxesPacketPubSubType.class);
      PUB_SUB_TYPES.put(ObjectDetectorResultPacket.class, ObjectDetectorResultPacketPubSubType.class);
      PUB_SUB_TYPES.put(RequestStereoPointCloudMessage.class, RequestStereoPointCloudMessagePubSubType.class);
      PUB_SUB_TYPES.put(StereoVisionPointCloudMessage.class, StereoVisionPointCloudMessagePubSubType.class);
      PUB_SUB_TYPES.put(WholeBodyTrajectoryToolboxOutputStatus.class, WholeBodyTrajectoryToolboxOutputStatusPubSubType.class);

      mapHasBeenFilled = true;
   }
}