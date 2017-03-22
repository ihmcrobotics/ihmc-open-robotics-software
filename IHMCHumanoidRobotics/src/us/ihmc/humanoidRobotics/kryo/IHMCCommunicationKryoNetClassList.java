package us.ihmc.humanoidRobotics.kryo;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.BoundingBoxesPacket;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket;
import us.ihmc.communication.packets.HeatMapPacket;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.ObjectDetectorResultPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestLidarScanMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.SimulatedLidarScanPacket;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.LegCompliancePacket;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingAction;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingCommand;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingStatus;
import us.ihmc.humanoidRobotics.communication.packets.SCSListenerPacket;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorCommandPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeResponsePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ButtonData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DebrisData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DoorLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DrillPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorButtonPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorDebrisPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WallTaskBehaviorData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorStatusEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.BlindWalkingDirection;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.BlindWalkingSpeed;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.driving.DrivingStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.driving.DrivingTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeNodeMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasDesiredPumpPSIPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorAutoEnableFlagPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorEnablePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasWristSensorCalibrationRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.BatchedDesiredSteeringAngleAndSingleJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.CalibrateArmPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ControlStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.DesiredSteeringAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPowerCyclePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.SpigotPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.SteeringWheelInformationPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.TorusPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.AbstractPointCloudPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DrillDetectionPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.FilteredPointCloudPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.FisheyePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.HeadPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPointMapPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseMocapExperimentPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseParameterPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseTest;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseTest.MultisenseFrameName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.RawIMUPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.RequestWristForceSensorCalibrationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorModePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.TestbedClientPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.TestbedServerPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.UIConnectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.valkyrie.ValkyrieLowLevelControlModeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket.RequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.TrackingWeightsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MultiJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.SingleJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.remote.serialization.JointConfigurationData;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestMessage;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class IHMCCommunicationKryoNetClassList extends NetClassList
{
   public IHMCCommunicationKryoNetClassList()
   {
      registerPacketClass(Packet.class);

      registerPacketField(RequestType.class);
      registerPacketField(String.class);
      registerPacketField(char[].class);
      registerPacketField(String[].class);
      registerPacketClass(HandCollisionDetectedPacket.class);

      registerPacketField(MultisenseTest.class);
      registerPacketField(MultisenseFrameName.class);
      registerPacketClass(MultisenseMocapExperimentPacket.class);

      // Video data
      registerPacketClass(VideoPacket.class);
      registerPacketClass(SimulatedLidarScanPacket.class);
      registerPacketClass(FilteredPointCloudPacket.class);
      registerPacketClass(TestbedServerPacket.class);
      registerPacketClass(JointConfigurationData.class);

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
      registerPacketClass(HandRotateAboutAxisPacket.class);
      registerPacketField(HandRotateAboutAxisPacket.DataType.class);
      registerPacketClass(SteeringWheelInformationPacket.class);
      registerPacketField(RobotSide.class);
      registerPacketField(Point3D.class);
      registerPacketField(Point3D32.class);
      registerPacketField(Vector3D.class);
      registerPacketField(Vector2D32.class);
      registerPacketClass(DesiredSteeringAnglePacket.class);

      registerPacketClass(HandComplianceControlParametersMessage.class);
      registerPacketField(Vector3D32.class);
      registerPacketField(boolean[].class);

      // Endeffector load bearing message
      registerPacketClass(FootLoadBearingMessage.class);
      registerPacketClass(HandLoadBearingMessage.class);
      registerPacketClass(LoadBearingRequest.class);

      // User control mode
      registerPacketClass(ArmDesiredAccelerationsMessage.class);
      registerPacketClass(NeckDesiredAccelerationsMessage.class);

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
      registerPacketClass(WholeBodyTrajectoryMessage.class);
      registerPacketClass(PelvisHeightTrajectoryMessage.class);
      registerPacketClass(TrackingWeightsMessage.class);
      registerPacketClass(StopAllTrajectoryMessage.class);
      registerPacketClass(GoHomeMessage.class);

      // Trajectory message fields
      registerPacketClass(ExecutionMode.class);
      registerPacketClass(ExecutionTiming.class);
      registerPacketClass(OneDoFJointTrajectoryMessage.class);
      registerPacketClass(TrajectoryPoint1DMessage.class);
      registerPacketClass(EuclideanTrajectoryPointMessage.class);
      registerPacketClass(SO3TrajectoryPointMessage.class);
      registerPacketClass(SE3TrajectoryPointMessage.class);
      registerPacketClass(BodyPart.class);

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
      registerPacketField(BodyPart.class);

      // Controller failure
      registerPacketClass(WalkingControllerFailureStatusMessage.class);

      // Valkyrie specific
      registerPacketClass(ValkyrieLowLevelControlModeMessage.class);
      registerPacketField(ValkyrieLowLevelControlModeMessage.ControlMode.class);

      // Handstep
      registerPacketClass(HandstepPacket.class);

      //Vehicle
      registerPacketClass(VehiclePosePacket.class);

      // Torus pose
      registerPacketClass(TorusPosePacket.class);

      // Spigot pose
      registerPacketClass(SpigotPosePacket.class);

      // Toolbox modules
      registerPacketClass(ToolboxStateMessage.class);
      registerPacketField(ToolboxState.class);
      registerPacketClass(KinematicsToolboxOutputStatus.class);
      registerPacketClass(FootstepPlanningToolboxOutputStatus.class);
      registerPacketClass(FootstepPlanningRequestPacket.class);
      registerPacketClass(FootstepPlanningResult.class);

      // Joint data
      registerPacketClass(RobotConfigurationData.class);
      registerPacketFields(double[].class, Vector3D.class);
      registerPacketFields(DenseMatrix64F.class);
      registerPacketFields(DenseMatrix64F[].class);

      // Footstep data
      registerPacketClass(FootstepDataMessage.class);
      registerPacketClass(AdjustFootstepMessage.class);
      registerPacketField(FootstepOrigin.class);
      registerPacketField(ArrayList.class);

      registerPacketClass(FootstepDataListMessage.class);
      registerPacketField(ArrayList.class);

      registerPacketClass(BlindWalkingPacket.class);
      registerPacketFields(Point2D.class, BlindWalkingDirection.class, BlindWalkingSpeed.class);

      registerPacketClass(PauseWalkingMessage.class);
      registerPacketClass(FootstepStatus.class);
      registerPacketClass(WalkingStatusMessage.class);
      registerPacketClass(TrajectoryType.class);

      registerPacketField(ArrayList.class);
      registerPacketField(FootstepStatus.Status.class);
      registerPacketField(WalkingStatusMessage.Status.class);
      registerPacketClass(AbortWalkingMessage.class);

      // Planar regions
      registerPacketClass(PlanarRegionsListMessage.class);
      registerPacketClass(PlanarRegionMessage.class);
      registerPacketClass(RequestPlanarRegionsListMessage.class);
      registerPacketClass(RequestPlanarRegionsListMessage.RequestType.class);
      registerPacketField(Point3D32.class);
      registerPacketField(Vector3D32.class);
      registerPacketField(Point2D32.class);
      registerPacketField(Point2D32[].class);

      //SCS
      registerPacketClass(SCSListenerPacket.class);

      // LIDAR
      registerPacketClass(AbstractPointCloudPacket.class);
      registerPacketClass(DepthDataStateCommand.class);
      registerPacketClass(DepthDataClearCommand.class);
      registerPacketField(DepthDataTree.class);
      registerPacketField(LidarState.class);

      registerPacketField(int[].class);
      registerPacketField(float[].class);
      registerPacketField(Quaternion32.class);
      registerPacketField(Vector3D32.class);
      registerPacketField(LidarScanParameters.class);

      // Robot pose estimation
      registerPacketField(RigidBodyTransform.class);
      registerPacketField(RigidBodyTransform[].class);
      registerPacketClass(StampedPosePacket.class);

      //Mocap
      registerPacketClass(DetectedObjectPacket.class);

      // high levle state
      registerPacketClass(HighLevelStateMessage.class);
      registerPacketClass(HighLevelState.class);
      registerPacketClass(HighLevelStateChangeStatusMessage.class);

      // Recording
      registerPacketClass(EndOfScriptCommand.class);

      // Driving
      registerPacketClass(LowLevelDrivingCommand.class);
      registerPacketClass(LowLevelDrivingStatus.class);
      registerPacketClass(LowLevelDrivingCommand.class);
      registerPacketField(LowLevelDrivingAction.class);

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

      registerPacketClass(TestbedClientPacket.class);

      // registerPacketClass(FishEyeControlPacket.class);
      registerPacketClass(ControlStatusPacket.class);
      registerPacketField(ControlStatusPacket.ControlStatus.class);

      // Humanoid Behaviors
      registerPacketClass(HumanoidBehaviorTypePacket.class);
      registerPacketField(HumanoidBehaviorType.class);
      registerPacketClass(BehaviorControlModePacket.class);
      registerPacketField(BehaviorControlModeEnum.class);
      registerPacketClass(ScriptBehaviorStatusPacket.class);
      registerPacketClass(ScriptBehaviorStatusEnum.class);
      registerPacketClass(BehaviorControlModeResponsePacket.class);
      registerPacketClass(ScriptBehaviorInputPacket.class);
      registerPacketClass(WallTaskBehaviorData.class);
      registerPacketField(WallTaskBehaviorData.Commands.class);

      registerPacketClass(DepthDataStateCommand.class);
      registerPacketClass(DepthDataClearCommand.class);
      registerPacketClass(DepthDataFilterParameters.class);
      registerPacketClass(DrivingTrajectoryPacket.class);
      registerPacketField(DrivingTrajectoryPacket.Length.class);
      registerPacketClass(DrivingStatePacket.class);
      registerPacketField(DrivingStatePacket.DrivingState.class);
      registerPacketClass(CalibrateArmPacket.class);
      registerPacketClass(HandDesiredConfigurationMessage.class);
      registerPacketClass(ManualHandControlPacket.class);
      registerPacketClass(MultisenseParameterPacket.class);
      registerPacketClass(TestbedClientPacket.class);
      registerPacketClass(SnapFootstepPacket.class);
      registerPacketClass(BlackFlyParameterPacket.class);
      registerPacketClass(HumanoidBehaviorDebrisPacket.class);
      registerPacketField(DebrisData.class);
      registerPacketClass(WalkToGoalBehaviorPacket.class);
      registerPacketField(WalkToGoalBehaviorPacket.WalkToGoalAction.class);
      registerPacketClass(FootstepPlanRequestPacket.class);
      registerPacketClass(DrillPacket.class);
      registerPacketClass(SimpleCoactiveBehaviorDataPacket.class);

      registerPacketClass(DoorLocationPacket.class);
      registerPacketClass(ValveLocationPacket.class);
      registerPacketClass(BehaviorStatusPacket.class);

      registerPacketClass(CapturabilityBasedStatus.class);
      registerPacketFields(Point2D.class, Point2D[].class);

      registerPacketClass(ButtonData.class);
      registerPacketClass(HumanoidBehaviorButtonPacket.class);

      // Planning
      registerPacketClass(FootstepPathPlanPacket.class);

      // Localization
      registerPacketClass(LocalizationPacket.class);
      registerPacketClass(LocalizationStatusPacket.class);
      registerPacketClass(PelvisPoseErrorPacket.class);
      registerPacketClass(LocalizationPointMapPacket.class);

      registerPacketClass(RawIMUPacket.class);
      registerPacketClass(HeadPosePacket.class);
      registerPacketClass(HeadPosePacket.MeasurementStatus.class);

      registerPacketClass(JointAnglesPacket.class);
      registerPacketClass(SingleJointAnglePacket.class);
      registerPacketField(SingleJointAnglePacket[].class);
      registerPacketClass(MultiJointAnglePacket.class);

      registerPacketField(Vector3D[].class);
      registerPacketField(Quaternion[].class);
      registerPacketField(Point3D[].class);

      registerPacketClass(PointCloudWorldPacket.class);
      registerPacketClass(LidarPosePacket.class);

      // New quadTree
      registerPacketClass(HeightQuadTreeToolboxRequestMessage.class);
      registerPacketClass(HeightQuadTreeMessage.class);
      registerPacketField(HeightQuadTreeNodeMessage.class);
      registerPacketField(HeightQuadTreeNodeMessage[].class);

      // Lidar messages
      registerPacketClass(LidarScanMessage.class);
      registerPacketClass(RequestLidarScanMessage.class);

      registerPacketClass(ControllerCrashNotificationPacket.class);
      registerPacketField(ControllerCrashNotificationPacket.CrashLocation.class);
      registerPacketClass(InvalidPacketNotificationPacket.class);

      registerPacketClass(AtlasWristSensorCalibrationRequestPacket.class);
      registerPacketClass(AtlasElectricMotorEnablePacket.class);
      registerPacketField(AtlasElectricMotorPacketEnum.class);
      registerPacketClass(AtlasElectricMotorAutoEnableFlagPacket.class);

      registerPacketField(RobotMotionStatus.class);

      registerPacketField(AuxiliaryRobotData.class);
      registerPacketField(AtlasAuxiliaryRobotData.class);
      registerPacketField(long[].class);
      registerPacketField(boolean[].class);
      registerPacketField(float[].class);
      registerPacketField(float[][].class);

      registerPacketClass(AtlasDesiredPumpPSIPacket.class);

      registerPacketClass(StateEstimatorModePacket.class);
      registerPacketField(StateEstimatorModePacket.StateEstimatorMode.class);

      registerPacketClass(RequestWristForceSensorCalibrationPacket.class);
      registerPacketClass(UIConnectedPacket.class);
      registerPacketClass(LegCompliancePacket.class);
      registerPacketClass(DrillDetectionPacket.class);
      registerPacketClass(UIPositionCheckerPacket.class);
      registerPacketClass(BatchedDesiredSteeringAngleAndSingleJointAnglePacket.class);
      registerPacketClass(TextToSpeechPacket.class);
      registerPacketField(VideoSource.class);

      registerPacketField(Quaternion.class);

      registerPacketClass(HeatMapPacket.class);
      registerPacketClass(BoundingBoxesPacket.class);

      registerPacketClass(ObjectDetectorResultPacket.class);

   }
}
