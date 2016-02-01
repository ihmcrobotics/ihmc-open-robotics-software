package us.ihmc.humanoidRobotics.kryo;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.SimulatedLidarScanPacket;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidRobotics.communication.packets.BumStatePacket;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangePacket;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.humanoidRobotics.communication.packets.LegCompliancePacket;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingAction;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingCommand;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingStatus;
import us.ihmc.humanoidRobotics.communication.packets.SCSListenerPacket;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorCommandPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ButtonData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DebrisData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DrillPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorButtonPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeResponsePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorDebrisPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.TurnValvePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WallTaskBehaviorData;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorStatusEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.BlindWalkingDirection;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.BlindWalkingSpeed;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.driving.DrivingStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.driving.DrivingTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasDesiredPumpPSIPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorAutoEnableFlagPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorEnablePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasWristSensorCalibrationRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.BatchedDesiredSteeringAngleAndSingleJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.CalibrateArmPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ControlStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.DesiredSteeringAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPowerCyclePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.JointTrajectoryPoint;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.SpigotPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.SteeringWheelInformationPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopMotionPacket;
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
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPointMapPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LookAtPacket;
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
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket.RequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ThighStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MultiJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.SingleJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryDevelopmentPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.remote.serialization.JointConfigurationData;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import boofcv.struct.calib.IntrinsicParameters;

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
      
      registerPacketField(IMUPacket[].class);
      registerPacketField(IMUPacket.class);
      
      registerPacketField(byte[].class);
      registerPacketField(Point3d.class);
      registerPacketField(Quat4d.class);
      registerPacketField(TimeStampedTransform3D.class);
      
      registerPacketField(PacketDestination.class);
      
      // Hand pose
      registerPacketClass(HandPosePacket.class);
      registerPacketClass(StopMotionPacket.class);
      registerPacketClass(AutomaticManipulationAbortPacket.class);
      registerPacketClass(ManipulationAbortedStatus.class);
      registerPacketClass(HandStatePacket.class);
      registerPacketClass(FingerStatePacket.class);
      registerPacketField(FingerState.class);
      registerPacketClass(HandLoadBearingPacket.class);
      registerPacketFields(RobotSide.class);
      registerPacketFields(HandPosePacket.Frame.class);
      registerPacketFields(HandPosePacket.DataType.class);
      registerPacketClass(ObjectWeightPacket.class);
      registerPacketClass(HandRotateAboutAxisPacket.class);
      registerPacketField(HandRotateAboutAxisPacket.DataType.class);
      registerPacketClass(SteeringWheelInformationPacket.class);
      registerPacketField(RobotSide.class);
      registerPacketField(Point3d.class);
      registerPacketField(Vector3d.class);
      registerPacketClass(DesiredSteeringAnglePacket.class);

      registerPacketClass(HandComplianceControlParametersPacket.class);
      registerPacketField(Vector3f.class);
      registerPacketField(boolean[].class);

      // Hand pose list
      registerPacketClass(HandPoseListPacket.class);
      registerPacketField(double[][].class);

      // Handstep
      registerPacketClass(HandstepPacket.class);

      //Foot pose (similar to hand pose)
      registerPacketClass(FootPosePacket.class);

      //Vehicle
      registerPacketClass(VehiclePosePacket.class);

      // Torus pose
      registerPacketClass(TorusPosePacket.class);

      // Spigot pose
      registerPacketClass(SpigotPosePacket.class);

      //Chest Orientation
      registerPacketClass(ChestOrientationPacket.class);

      // thigh and bum
      registerPacketClass(ThighStatePacket.class);
      registerPacketClass(BumStatePacket.class);

      // Joint data
      registerPacketClass(RobotConfigurationData.class);
      registerPacketFields(double[].class, Vector3d.class);
      registerPacketFields(DenseMatrix64F.class);
      registerPacketFields(DenseMatrix64F[].class);
      
      // Footstep data
      registerPacketClass(FootstepData.class);
      registerPacketField(ArrayList.class);

      registerPacketClass(FootstepDataList.class);
      registerPacketField(ArrayList.class);

      registerPacketClass(FootStatePacket.class);
      
      registerPacketClass(BlindWalkingPacket.class);
      registerPacketFields(Point2d.class, BlindWalkingDirection.class, BlindWalkingSpeed.class);

      registerPacketClass(PauseCommand.class);
      registerPacketClass(FootstepStatus.class);
      registerPacketClass(TrajectoryType.class);

      
      registerPacketField(ArrayList.class);
      registerPacketField(FootstepStatus.Status.class);
      registerPacketClass(AbortWalkingPacket.class);
      
      // Head orientation
      registerPacketClass(HeadOrientationPacket.class);
      registerPacketClass(LookAtPacket.class);
      registerPacketClass(PelvisPosePacket.class);

      // Com Height
      registerPacketClass(ComHeightPacket.class);

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
      registerPacketField(Quat4f.class);
      registerPacketField(Vector3f.class);
      registerPacketField(LidarScanParameters.class);
      
      // Robot pose estimation
      registerPacketField(RigidBodyTransform.class);
      registerPacketField(RigidBodyTransform[].class);
      registerPacketClass(StampedPosePacket.class);
      
      //Mocap
      registerPacketClass(DetectedObjectPacket.class);

      // high levle state
      registerPacketClass(HighLevelStatePacket.class);
      registerPacketClass(HighLevelState.class);
      registerPacketClass(HighLevelStateChangePacket.class);
            
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
      registerPacketClass(FingerStatePacket.class);
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
      registerPacketClass(TurnValvePacket.class);
      
      registerPacketClass(CapturabilityBasedStatus.class);
      registerPacketFields(Point2d.class, Point2d[].class);
      
      registerPacketClass(ButtonData.class);
      registerPacketClass(HumanoidBehaviorButtonPacket.class);
      
      // Planning
      registerPacketClass(FootstepPathPlanPacket.class);
      
      // Localization
      registerPacketClass(LocalizationPacket.class);
      registerPacketClass(LocalizationStatusPacket.class);
      registerPacketClass(PelvisPoseErrorPacket.class);
      registerPacketClass(LocalizationPointMapPacket.class);
      
      registerPacketClass(HandPoseStatus.class);
//      registerPacketField(HandPoseStatus.Status.class);
      registerPacketClass(RawIMUPacket.class);
      registerPacketClass(HeadPosePacket.class);
      registerPacketClass(HeadPosePacket.MeasurementStatus.class);
      
      registerPacketClass(ArmJointTrajectoryPacket.class);
      registerPacketField(JointTrajectoryPoint.class);
      registerPacketField(JointTrajectoryPoint[].class);
      registerPacketClass(JointTrajectoryPoint.class);
      
      //Whole Body IK 
      registerPacketClass(WholeBodyTrajectoryDevelopmentPacket.class);
      registerPacketClass(WholeBodyTrajectoryPacket.class);
      
      registerPacketClass(JointAnglesPacket.class);
      registerPacketClass(SingleJointAnglePacket.class);
      registerPacketField(SingleJointAnglePacket[].class);
      registerPacketClass(MultiJointAnglePacket.class);
      
      registerPacketField(Vector3d[].class);
      registerPacketField(Quat4d[].class);
      registerPacketField(Point3d[].class);
      
      registerPacketClass(PointCloudWorldPacket.class);
      
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
      
      registerPacketClass(BatchedDesiredSteeringAngleAndSingleJointAnglePacket.class);
      registerPacketClass(TextToSpeechPacket.class);
   }
}
