package us.ihmc.humanoidRobotics.kryo;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalAction;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.idl.RecyclingArrayListPubSub;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class IHMCCommunicationKryoNetClassList extends NetClassList
{
   public IHMCCommunicationKryoNetClassList()
   {
      registerPacketField(TDoubleArrayList.class);
      registerPacketField(TIntArrayList.class);
      registerPacketField(TLongArrayList.class);
      registerPacketField(TFloatArrayList.class);
      registerPacketField(TByteArrayList.class);
      registerPacketField(RecyclingArrayListPubSub.class);

      registerPacketClass(Packet.class);

      registerPacketField(StringBuilder.class);
      registerPacketField(StringBuilder[].class);

      registerPacketField(FootstepPlanRequestType.class);
      registerPacketField(char[].class);
      registerPacketClass(HandCollisionDetectedPacket.class);

      // Video data
      registerPacketClass(VideoPacket.class);
      registerPacketField(VideoPacket.class);
      registerPacketClass(SimulatedLidarScanPacket.class);
      registerPacketField(IntrinsicParametersMessage.class);
      registerPacketField(LidarScanParametersMessage.class);

      registerPacketClass(IMUPacket.class);
      registerPacketField(IMUPacket[].class);
      registerPacketField(IMUPacket.class);

      registerPacketField(byte[].class);
      registerPacketField(Point3D.class);
      registerPacketField(Quaternion.class);
      registerPacketField(TimeStampedTransform3D.class);

      registerPacketField(PacketDestination.class);

      // Hand pose
      registerPacketClass(AutomaticManipulationAbortMessage.class);
      registerPacketClass(ManipulationAbortedStatus.class);
      registerPacketClass(HandDesiredConfigurationMessage.class);
      registerPacketField(HandConfiguration.class);
      registerPacketFields(RobotSide.class);
      registerPacketClass(ObjectWeightPacket.class);
      registerPacketField(RobotSide.class);
      registerPacketField(Point3D.class);
      registerPacketField(Point3D32.class);
      registerPacketField(Vector3D.class);

      registerPacketField(Vector3D32.class);
      registerPacketField(Vector3D32[].class);
      registerPacketField(boolean[].class);

      // Endeffector load bearing message
      registerPacketClass(FootLoadBearingMessage.class);
      registerPacketClass(HandLoadBearingMessage.class);
      registerPacketField(LoadBearingRequest.class);
      registerPacketField(LoadBearingMessage.class);

      // User control mode
      registerPacketClass(ArmDesiredAccelerationsMessage.class);
      registerPacketClass(NeckDesiredAccelerationsMessage.class);
      registerPacketField(DesiredAccelerationsMessage.class);

      // Trajectory messages
      registerPacketClass(HandTrajectoryMessage.class);
      registerPacketClass(ArmTrajectoryMessage.class);
      registerPacketClass(HeadTrajectoryMessage.class);
      registerPacketClass(NeckTrajectoryMessage.class);
      registerPacketClass(ChestTrajectoryMessage.class);
      registerPacketClass(SpineTrajectoryMessage.class);
      registerPacketClass(SpineDesiredAccelerationsMessage.class);
      registerPacketClass(PelvisTrajectoryMessage.class);
      registerPacketClass(PelvisOrientationTrajectoryMessage.class);
      registerPacketClass(FootTrajectoryMessage.class);
      registerPacketClass(MessageOfMessages.class);
      registerPacketClass(WholeBodyTrajectoryMessage.class);
      registerPacketClass(PelvisHeightTrajectoryMessage.class);
      registerPacketClass(StopAllTrajectoryMessage.class);
      registerPacketClass(GoHomeMessage.class);
      registerPacketClass(HandHybridJointspaceTaskspaceTrajectoryMessage.class);
      registerPacketClass(ChestHybridJointspaceTaskspaceTrajectoryMessage.class);
      registerPacketClass(HeadHybridJointspaceTaskspaceTrajectoryMessage.class);
      registerPacketClass(ClearDelayQueueMessage.class);
      registerPacketClass(MomentumTrajectoryMessage.class);
      registerPacketClass(CenterOfMassTrajectoryMessage.class);

      // Trajectory message fields
      registerPacketField(HandTrajectoryMessage.class);
      registerPacketField(ArmTrajectoryMessage.class);
      registerPacketField(ChestTrajectoryMessage.class);
      registerPacketField(PelvisTrajectoryMessage.class);
      registerPacketField(FootTrajectoryMessage.class);
      registerPacketField(HeadTrajectoryMessage.class);
      registerPacketField(SE3TrajectoryMessage.class);
      registerPacketField(SO3TrajectoryMessage.class);
      registerPacketField(EuclideanTrajectoryMessage.class);
      registerPacketField(FrameInformation.class);
      registerPacketField(JointspaceTrajectoryMessage.class);
      registerPacketField(QueueableMessage.class);
      registerPacketField(ExecutionMode.class);
      registerPacketField(ExecutionTiming.class);
      registerPacketClass(OneDoFJointTrajectoryMessage.class);
      registerPacketClass(TrajectoryPoint1DMessage.class);
      registerPacketClass(EuclideanTrajectoryPointMessage.class);
      registerPacketClass(SO3TrajectoryPointMessage.class);
      registerPacketClass(SE3TrajectoryPointMessage.class);

      registerPacketField(OneDoFJointTrajectoryMessage.class);
      registerPacketField(OneDoFJointTrajectoryMessage[].class);
      registerPacketField(TrajectoryPoint1DMessage.class);
      registerPacketField(TrajectoryPoint1DMessage[].class);
      registerPacketField(EuclideanTrajectoryPointMessage.class);
      registerPacketField(EuclideanTrajectoryPointMessage[].class);
      registerPacketField(SO3TrajectoryPointMessage.class);
      registerPacketField(SO3TrajectoryPointMessage[].class);
      registerPacketField(SE3TrajectoryPointMessage.class);
      registerPacketField(SE3TrajectoryPointMessage[].class);
      registerPacketField(SelectionMatrix3DMessage.class);
      registerPacketField(WeightMatrix3DMessage.class);
      registerPacketField(HumanoidBodyPart.class);
      registerPacketField(Class.class);

      // Controller failure
      registerPacketClass(WalkingControllerFailureStatusMessage.class);

      //Vehicle
      registerPacketClass(VehiclePosePacket.class);

      // Toolbox modules
      registerPacketClass(ToolboxStateMessage.class);
      registerPacketField(ToolboxState.class);
      registerPacketClass(FootstepPlanningToolboxOutputStatus.class);
      registerPacketClass(FootstepPlanningRequestPacket.class);
      registerPacketField(FootstepPlanningResult.class);
      registerPacketField(Pose2D.class);
      registerPacketField(Point2D.class);
      registerPacketField(Vector2D.class);
      registerPacketField(Orientation2D.class);
      registerPacketField(Point3D.class);
      registerPacketField(Point3D[].class);
      registerPacketField(Point3D[][].class);

      // Packets for the kinematics toolbox
      registerPacketClass(KinematicsToolboxOutputStatus.class);
      registerPacketClass(KinematicsToolboxCenterOfMassMessage.class);
      registerPacketClass(KinematicsToolboxRigidBodyMessage.class);
      registerPacketClass(KinematicsToolboxConfigurationMessage.class);
      registerPacketClass(HumanoidKinematicsToolboxConfigurationMessage.class);
      registerPacketField(KinematicsToolboxOutputStatus.class);
      registerPacketField(Pose3D.class);
      registerPacketField(Pose3D[].class);

      // Packets for whole-body trajectory toolbox
      registerPacketClass(WholeBodyTrajectoryToolboxMessage.class);
      registerPacketClass(WholeBodyTrajectoryToolboxConfigurationMessage.class);
      registerPacketClass(WaypointBasedTrajectoryMessage.class);
      registerPacketClass(ReachingManifoldMessage.class);
      registerPacketClass(RigidBodyExplorationConfigurationMessage.class);
      registerPacketFields(WholeBodyTrajectoryToolboxConfigurationMessage.class);
      registerPacketFields(WaypointBasedTrajectoryMessage.class);
      registerPacketFields(WaypointBasedTrajectoryMessage[].class);
      registerPacketFields(ReachingManifoldMessage.class);
      registerPacketFields(ReachingManifoldMessage[].class);
      registerPacketFields(RigidBodyExplorationConfigurationMessage.class);
      registerPacketFields(RigidBodyExplorationConfigurationMessage[].class);
      registerPacketFields(ConfigurationSpaceName.class);
      registerPacketFields(ConfigurationSpaceName[].class);

      // Joint data
      registerPacketClass(RobotConfigurationData.class);
      registerPacketClass(AtlasAuxiliaryRobotData.class);
      registerPacketFields(double[].class, Vector3D.class);
      registerPacketFields(DenseMatrix64F.class);
      registerPacketFields(DenseMatrix64F[].class);
      registerPacketField(SpatialVectorMessage.class);
      registerPacketField(SpatialVectorMessage[].class);

      // Footstep data
      registerPacketClass(FootstepDataMessage.class);
      registerPacketClass(AdjustFootstepMessage.class);
      registerPacketField(ArrayList.class);
      registerPacketField(List.class);

      registerPacketField(FootstepDataListMessage.class);
      registerPacketClass(FootstepDataListMessage.class);
      registerPacketField(ArrayList.class);

      registerPacketClass(PauseWalkingMessage.class);
      registerPacketClass(FootstepStatusMessage.class);
      registerPacketClass(PlanOffsetStatus.class);
      registerPacketClass(WalkingStatusMessage.class);
      registerPacketField(TrajectoryType.class);
      registerPacketField(StringBuilder.class);
      registerPacketField(StringBuilder[].class);

      registerPacketField(ArrayList.class);
      registerPacketField(FootstepStatus.class);
      registerPacketField(WalkingStatus.class);
      registerPacketClass(AbortWalkingMessage.class);
      registerPacketClass(PrepareForLocomotionMessage.class);

      // Planar regions
      registerPacketClass(PlanarRegionsListMessage.class);
      registerPacketField(PlanarRegionsListMessage.class);
      registerPacketClass(PlanarRegionMessage.class);
      registerPacketField(PlanarRegionMessage.class);
      registerPacketField(PlanarRegionMessage[].class);
      registerPacketClass(RequestPlanarRegionsListMessage.class);
      registerPacketField(PlanarRegionsRequestType.class);
      registerPacketField(Point3D32.class);
      registerPacketField(Vector3D32.class);
      registerPacketField(BoundingBox3DMessage.class);
      registerPacketField(Polygon2DMessage.class);
      registerPacketField(Polygon2DMessage[].class);

      //SCS
      registerPacketClass(SCSListenerPacket.class);

      registerPacketField(int[].class);
      registerPacketField(float[].class);
      registerPacketField(Quaternion32.class);
      registerPacketField(Vector3D32.class);
      registerPacketField(LidarScanParameters.class);

      // Robot pose estimation
      registerPacketClass(StampedPosePacket.class);

      //Mocap
      registerPacketClass(DetectedObjectPacket.class);

      // high levle state
      registerPacketClass(HighLevelStateMessage.class);
      registerPacketField(HighLevelControllerName.class);
      registerPacketClass(HighLevelStateChangeStatusMessage.class);

      // Recording
      registerPacketClass(EndOfScriptCommand.class);

      //hand joint and control packets
      registerPacketClass(ManualHandControlPacket.class);
      registerPacketClass(HandPowerCyclePacket.class);
      registerPacketClass(HandJointAnglePacket.class);

      registerPacketClass(BDIBehaviorCommandPacket.class);
      registerPacketField(BDIRobotBehavior.class);
      registerPacketClass(BDIBehaviorStatusPacket.class);

      // Camera information related
      registerPacketField(IntrinsicParameters.class);

      registerPacketClass(FisheyePacket.class);

      registerPacketClass(MultisenseParameterPacket.class);

      // Humanoid Behaviors
      registerPacketClass(HumanoidBehaviorTypePacket.class);
      registerPacketField(HumanoidBehaviorType.class);
      registerPacketClass(BehaviorControlModePacket.class);
      registerPacketField(BehaviorControlModeEnum.class);
      registerPacketClass(BehaviorControlModeResponsePacket.class);

      registerPacketClass(HandDesiredConfigurationMessage.class);
      registerPacketClass(ManualHandControlPacket.class);
      registerPacketClass(MultisenseParameterPacket.class);
      registerPacketClass(SnapFootstepPacket.class);
      registerPacketClass(BlackFlyParameterPacket.class);
      registerPacketClass(WalkToGoalBehaviorPacket.class);
      registerPacketField(WalkToGoalAction.class);
      registerPacketClass(FootstepPlanRequestPacket.class);
      registerPacketField(FootstepDataMessage.class);
      registerPacketField(FootstepDataMessage[].class);
      registerPacketField(FootstepPlannerType.class);
      registerPacketClass(SimpleCoactiveBehaviorDataPacket.class);
      registerPacketClass(WallPosePacket.class);
      registerPacketClass(WalkOverTerrainGoalPacket.class);

      registerPacketClass(DoorLocationPacket.class);
      registerPacketClass(ValveLocationPacket.class);
      registerPacketClass(BehaviorStatusPacket.class);
      registerPacketField(CurrentBehaviorStatus.class);

      registerPacketClass(CapturabilityBasedStatus.class);

      // Planning
      registerPacketClass(FootstepPathPlanPacket.class);

      // Localization
      registerPacketClass(LocalizationPacket.class);
      registerPacketClass(LocalizationStatusPacket.class);
      registerPacketClass(PelvisPoseErrorPacket.class);
      registerPacketClass(LocalizationPointMapPacket.class);

      registerPacketField(Vector3D[].class);
      registerPacketField(Quaternion[].class);
      registerPacketField(Point3D[].class);

      registerPacketClass(PointCloudWorldPacket.class);

      // New quadTree
      registerPacketClass(HeightQuadTreeToolboxRequestMessage.class);
      registerPacketClass(HeightQuadTreeMessage.class);
      registerPacketField(Point3D32.class);
      registerPacketField(HeightQuadTreeLeafMessage.class);
      registerPacketField(HeightQuadTreeLeafMessage[].class);

      // Lidar messages
      registerPacketClass(LidarScanMessage.class);
      registerPacketClass(RequestLidarScanMessage.class);

      registerPacketClass(ControllerCrashNotificationPacket.class);
      registerPacketField(ControllerCrashLocation.class);
      registerPacketClass(InvalidPacketNotificationPacket.class);

      registerPacketClass(AtlasLowLevelControlModeMessage.class);
      registerPacketField(AtlasLowLevelControlMode.class);
      registerPacketClass(AtlasWristSensorCalibrationRequestPacket.class);
      registerPacketClass(AtlasElectricMotorEnablePacket.class);
      registerPacketField(AtlasElectricMotorPacketEnum.class);
      registerPacketClass(AtlasElectricMotorAutoEnableFlagPacket.class);

      registerPacketField(RobotMotionStatus.class);

      registerPacketField(long[].class);
      registerPacketField(boolean[].class);
      registerPacketField(float[].class);
      registerPacketField(float[][].class);

      registerPacketClass(AtlasDesiredPumpPSIPacket.class);

      registerPacketClass(StateEstimatorModePacket.class);
      registerPacketField(StateEstimatorMode.class);

      registerPacketClass(RequestWristForceSensorCalibrationPacket.class);
      registerPacketClass(UIConnectedPacket.class);
      registerPacketClass(LegCompliancePacket.class);
      registerPacketClass(DrillDetectionPacket.class);
      registerPacketClass(UIPositionCheckerPacket.class);
      registerPacketClass(TextToSpeechPacket.class);
      registerPacketField(VideoSource.class);

      registerPacketField(Quaternion.class);

      registerPacketClass(HeatMapPacket.class);
      registerPacketClass(BoundingBoxesPacket.class);

      registerPacketClass(ObjectDetectorResultPacket.class);
      registerPacketClass(RequestStereoPointCloudMessage.class);
      registerPacketClass(StereoVisionPointCloudMessage.class);
      registerPacketField(HeatMapPacket.class);
      registerPacketField(BoundingBoxesPacket.class);

      registerPacketClass(WholeBodyTrajectoryToolboxOutputStatus.class);
      registerPacketField(KinematicsToolboxOutputStatus[].class);
   }
}
