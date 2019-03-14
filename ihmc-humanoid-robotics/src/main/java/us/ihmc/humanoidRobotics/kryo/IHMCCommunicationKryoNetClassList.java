package us.ihmc.humanoidRobotics.kryo;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import actionlib_msgs.msg.dds.GoalIDPubSubType;
import actionlib_msgs.msg.dds.GoalStatusArrayPubSubType;
import actionlib_msgs.msg.dds.GoalStatusPubSubType;
import boofcv.struct.calib.IntrinsicParameters;
import builtin_interfaces.msg.dds.DurationPubSubType;
import builtin_interfaces.msg.dds.TimePubSubType;
import controller_msgs.msg.dds.*;
import diagnostic_msgs.msg.dds.DiagnosticArrayPubSubType;
import diagnostic_msgs.msg.dds.DiagnosticStatusPubSubType;
import diagnostic_msgs.msg.dds.KeyValuePubSubType;
import geometry_msgs.msg.dds.AccelPubSubType;
import geometry_msgs.msg.dds.AccelStampedPubSubType;
import geometry_msgs.msg.dds.AccelWithCovariancePubSubType;
import geometry_msgs.msg.dds.AccelWithCovarianceStampedPubSubType;
import geometry_msgs.msg.dds.InertiaPubSubType;
import geometry_msgs.msg.dds.InertiaStampedPubSubType;
import geometry_msgs.msg.dds.Point32PubSubType;
import geometry_msgs.msg.dds.PointPubSubType;
import geometry_msgs.msg.dds.PointStampedPubSubType;
import geometry_msgs.msg.dds.PolygonPubSubType;
import geometry_msgs.msg.dds.PolygonStampedPubSubType;
import geometry_msgs.msg.dds.Pose2DPubSubType;
import geometry_msgs.msg.dds.PoseArrayPubSubType;
import geometry_msgs.msg.dds.PosePubSubType;
import geometry_msgs.msg.dds.PoseStampedPubSubType;
import geometry_msgs.msg.dds.PoseWithCovariancePubSubType;
import geometry_msgs.msg.dds.PoseWithCovarianceStampedPubSubType;
import geometry_msgs.msg.dds.QuaternionPubSubType;
import geometry_msgs.msg.dds.QuaternionStampedPubSubType;
import geometry_msgs.msg.dds.TransformPubSubType;
import geometry_msgs.msg.dds.TransformStampedPubSubType;
import geometry_msgs.msg.dds.TwistPubSubType;
import geometry_msgs.msg.dds.TwistStampedPubSubType;
import geometry_msgs.msg.dds.TwistWithCovariancePubSubType;
import geometry_msgs.msg.dds.TwistWithCovarianceStampedPubSubType;
import geometry_msgs.msg.dds.Vector3PubSubType;
import geometry_msgs.msg.dds.Vector3StampedPubSubType;
import geometry_msgs.msg.dds.Wrench;
import geometry_msgs.msg.dds.WrenchPubSubType;
import geometry_msgs.msg.dds.WrenchStampedPubSubType;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import lifecycle_msgs.msg.dds.StatePubSubType;
import lifecycle_msgs.msg.dds.TransitionDescriptionPubSubType;
import lifecycle_msgs.msg.dds.TransitionEventPubSubType;
import lifecycle_msgs.msg.dds.TransitionPubSubType;
import nav_msgs.msg.dds.GridCellsPubSubType;
import nav_msgs.msg.dds.MapMetaDataPubSubType;
import nav_msgs.msg.dds.OccupancyGridPubSubType;
import nav_msgs.msg.dds.OdometryPubSubType;
import nav_msgs.msg.dds.PathPubSubType;
import rcl_interfaces.msg.dds.IntraProcessMessagePubSubType;
import rcl_interfaces.msg.dds.ListParametersResultPubSubType;
import rcl_interfaces.msg.dds.ParameterDescriptorPubSubType;
import rcl_interfaces.msg.dds.ParameterEventDescriptorsPubSubType;
import rcl_interfaces.msg.dds.ParameterEventPubSubType;
import rcl_interfaces.msg.dds.ParameterPubSubType;
import rcl_interfaces.msg.dds.ParameterTypePubSubType;
import rcl_interfaces.msg.dds.ParameterValuePubSubType;
import rcl_interfaces.msg.dds.SetParametersResultPubSubType;
import sensor_msgs.msg.dds.BatteryStatePubSubType;
import sensor_msgs.msg.dds.CameraInfoPubSubType;
import sensor_msgs.msg.dds.ChannelFloat32PubSubType;
import sensor_msgs.msg.dds.CompressedImagePubSubType;
import sensor_msgs.msg.dds.FluidPressurePubSubType;
import sensor_msgs.msg.dds.IlluminancePubSubType;
import sensor_msgs.msg.dds.ImagePubSubType;
import sensor_msgs.msg.dds.ImuPubSubType;
import sensor_msgs.msg.dds.JointStatePubSubType;
import sensor_msgs.msg.dds.JoyFeedbackArrayPubSubType;
import sensor_msgs.msg.dds.JoyFeedbackPubSubType;
import sensor_msgs.msg.dds.JoyPubSubType;
import sensor_msgs.msg.dds.LaserEchoPubSubType;
import sensor_msgs.msg.dds.LaserScanPubSubType;
import sensor_msgs.msg.dds.MagneticFieldPubSubType;
import sensor_msgs.msg.dds.MultiDOFJointStatePubSubType;
import sensor_msgs.msg.dds.MultiEchoLaserScanPubSubType;
import sensor_msgs.msg.dds.NavSatFixPubSubType;
import sensor_msgs.msg.dds.NavSatStatusPubSubType;
import sensor_msgs.msg.dds.PointCloud2PubSubType;
import sensor_msgs.msg.dds.PointCloudPubSubType;
import sensor_msgs.msg.dds.PointFieldPubSubType;
import sensor_msgs.msg.dds.RangePubSubType;
import sensor_msgs.msg.dds.RegionOfInterestPubSubType;
import sensor_msgs.msg.dds.RelativeHumidityPubSubType;
import sensor_msgs.msg.dds.TemperaturePubSubType;
import sensor_msgs.msg.dds.TimeReferencePubSubType;
import shape_msgs.msg.dds.MeshPubSubType;
import shape_msgs.msg.dds.MeshTrianglePubSubType;
import shape_msgs.msg.dds.PlanePubSubType;
import shape_msgs.msg.dds.SolidPrimitivePubSubType;
import std_msgs.msg.dds.BoolPubSubType;
import std_msgs.msg.dds.ByteMultiArrayPubSubType;
import std_msgs.msg.dds.BytePubSubType;
import std_msgs.msg.dds.CharPubSubType;
import std_msgs.msg.dds.ColorRGBAPubSubType;
import std_msgs.msg.dds.EmptyPubSubType;
import std_msgs.msg.dds.Float32MultiArrayPubSubType;
import std_msgs.msg.dds.Float32PubSubType;
import std_msgs.msg.dds.Float64MultiArrayPubSubType;
import std_msgs.msg.dds.Float64PubSubType;
import std_msgs.msg.dds.HeaderPubSubType;
import std_msgs.msg.dds.Int16MultiArrayPubSubType;
import std_msgs.msg.dds.Int16PubSubType;
import std_msgs.msg.dds.Int32MultiArrayPubSubType;
import std_msgs.msg.dds.Int32PubSubType;
import std_msgs.msg.dds.Int64MultiArrayPubSubType;
import std_msgs.msg.dds.Int64PubSubType;
import std_msgs.msg.dds.Int8MultiArrayPubSubType;
import std_msgs.msg.dds.Int8PubSubType;
import std_msgs.msg.dds.MultiArrayDimensionPubSubType;
import std_msgs.msg.dds.MultiArrayLayoutPubSubType;
import std_msgs.msg.dds.StringPubSubType;
import std_msgs.msg.dds.UInt16MultiArrayPubSubType;
import std_msgs.msg.dds.UInt16PubSubType;
import std_msgs.msg.dds.UInt32MultiArrayPubSubType;
import std_msgs.msg.dds.UInt32PubSubType;
import std_msgs.msg.dds.UInt64MultiArrayPubSubType;
import std_msgs.msg.dds.UInt64PubSubType;
import std_msgs.msg.dds.UInt8MultiArrayPubSubType;
import std_msgs.msg.dds.UInt8PubSubType;
import stereo_msgs.msg.dds.DisparityImagePubSubType;
import test_msgs.msg.dds.BoundedArrayNestedPubSubType;
import test_msgs.msg.dds.BoundedArrayPrimitivesPubSubType;
import test_msgs.msg.dds.BuiltinsPubSubType;
import test_msgs.msg.dds.DynamicArrayNestedPubSubType;
import test_msgs.msg.dds.DynamicArrayPrimitivesNestedPubSubType;
import test_msgs.msg.dds.DynamicArrayPrimitivesPubSubType;
import test_msgs.msg.dds.NestedPubSubType;
import test_msgs.msg.dds.PrimitivesPubSubType;
import test_msgs.msg.dds.StaticArrayNestedPubSubType;
import test_msgs.msg.dds.StaticArrayPrimitivesPubSubType;
import tf2_msgs.msg.dds.TF2ErrorPubSubType;
import tf2_msgs.msg.dds.TFMessagePubSubType;
import trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType;
import trajectory_msgs.msg.dds.JointTrajectoryPubSubType;
import trajectory_msgs.msg.dds.MultiDOFJointTrajectoryPointPubSubType;
import trajectory_msgs.msg.dds.MultiDOFJointTrajectoryPubSubType;
import us.ihmc.commons.lists.RecyclingArrayList;
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
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.types.ByteBufferPubSubType;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import visualization_msgs.msg.dds.ImageMarkerPubSubType;
import visualization_msgs.msg.dds.InteractiveMarkerControlPubSubType;
import visualization_msgs.msg.dds.InteractiveMarkerFeedbackPubSubType;
import visualization_msgs.msg.dds.InteractiveMarkerInitPubSubType;
import visualization_msgs.msg.dds.InteractiveMarkerPosePubSubType;
import visualization_msgs.msg.dds.InteractiveMarkerPubSubType;
import visualization_msgs.msg.dds.InteractiveMarkerUpdatePubSubType;
import visualization_msgs.msg.dds.MarkerArrayPubSubType;
import visualization_msgs.msg.dds.MarkerPubSubType;
import visualization_msgs.msg.dds.MenuEntryPubSubType;

public class IHMCCommunicationKryoNetClassList extends NetClassList
{
   public IHMCCommunicationKryoNetClassList()
   {
      registerPacketField(TDoubleArrayList.class);
      registerPacketField(TIntArrayList.class);
      registerPacketField(TLongArrayList.class);
      registerPacketField(TFloatArrayList.class);
      registerPacketField(TByteArrayList.class);
      registerPacketField(IDLSequence.Object.class);
      registerPacketField(IDLSequence.Float.class);
      registerPacketField(IDLSequence.Boolean.class);
      registerPacketField(IDLSequence.Double.class);
      registerPacketField(IDLSequence.Integer.class);
      registerPacketField(IDLSequence.Byte.class);
      registerPacketField(IDLSequence.Long.class);
      registerPacketField(IDLSequence.StringBuilderHolder.class);
      registerPacketField(TopicDataType.class);
      registerPacketField(RecyclingArrayList.class);
      registerPacketField(us.ihmc.idl.CDR.class);
      

      registerPacketField(WrenchTrajectoryMessage.class);
      registerPacketField(WrenchTrajectoryMessagePubSubType.class);
      registerPacketField(WrenchTrajectoryPointMessage.class);
      registerPacketField(WrenchTrajectoryPointMessage[].class);
      registerPacketField(WrenchTrajectoryPointMessagePubSubType.class);
      registerPacketField(KinematicsToolboxConfigurationMessagePubSubType.class);
      registerPacketField(Wrench.class);
      registerPacketField(WholeBodyTrajectoryToolboxMessagePubSubType.class);
      registerPacketField(PlanOffsetStatusPubSubType.class);
      registerPacketField(HeaderPubSubType.class);
      registerPacketField(SO3TrajectoryPointMessagePubSubType.class);
      registerPacketField(DetectedObjectPacketPubSubType.class);
      registerPacketField(StereoVisionPointCloudMessagePubSubType.class);
      registerPacketField(AtlasLowLevelControlModeMessagePubSubType.class);
      registerPacketField(PointStampedPubSubType.class);
      registerPacketField(AtlasElectricMotorAutoEnableFlagPacketPubSubType.class);
      registerPacketField(RigidBodyExplorationConfigurationMessagePubSubType.class);
      registerPacketField(RequestWristForceSensorCalibrationPacketPubSubType.class);
      registerPacketField(FootTrajectoryMessagePubSubType.class);
      registerPacketField(JoyPubSubType.class);
      registerPacketField(BehaviorStatusPacketPubSubType.class);
      registerPacketField(BehaviorControlModeResponsePacketPubSubType.class);
      registerPacketField(EuclideanTrajectoryPointMessagePubSubType.class);
      registerPacketField(JoyFeedbackArrayPubSubType.class);
      registerPacketField(ChestTrajectoryMessagePubSubType.class);
      registerPacketField(InertiaPubSubType.class);
      registerPacketField(BoolPubSubType.class);
      registerPacketField(FootstepStatusMessagePubSubType.class);
      registerPacketField(HandJointAnglePacketPubSubType.class);
      registerPacketField(Int16MultiArrayPubSubType.class);
      registerPacketField(RegionOfInterestPubSubType.class);
      registerPacketField(FootstepDataListMessagePubSubType.class);
      registerPacketField(MultiArrayDimensionPubSubType.class);
      registerPacketField(StaticArrayNestedPubSubType.class);
      registerPacketField(FisheyePacketPubSubType.class);
      registerPacketField(SpineTrajectoryMessagePubSubType.class);
      registerPacketField(RangePubSubType.class);
      registerPacketField(ListParametersResultPubSubType.class);
      registerPacketField(BuiltinsPubSubType.class);
      registerPacketField(MenuEntryPubSubType.class);
      registerPacketField(PlanePubSubType.class);
      registerPacketField(PrimitivesPubSubType.class);
      registerPacketField(StaticArrayPrimitivesPubSubType.class);
      registerPacketField(SpineDesiredAccelerationsMessagePubSubType.class);
      registerPacketField(FootLoadBearingMessagePubSubType.class);
      registerPacketField(ToolboxStateMessagePubSubType.class);
      registerPacketField(CameraInfoPubSubType.class);
      registerPacketField(RequestPlanarRegionsListMessagePubSubType.class);
      registerPacketField(EmptyPubSubType.class);
      registerPacketField(AbortWalkingMessagePubSubType.class);
      registerPacketField(PointCloud2PubSubType.class);
      registerPacketField(InteractiveMarkerControlPubSubType.class);
      registerPacketField(OdometryPubSubType.class);
      registerPacketField(WrenchStampedPubSubType.class);
      registerPacketField(TimePubSubType.class);
      registerPacketField(ParameterPubSubType.class);
      registerPacketField(UIConnectedPacketPubSubType.class);
      registerPacketField(WrenchPubSubType.class);
      registerPacketField(OccupancyGridPubSubType.class);
      registerPacketField(TFMessagePubSubType.class);
      registerPacketField(PathPubSubType.class);
      registerPacketField(LocalizationStatusPacketPubSubType.class);
      registerPacketField(UInt32MultiArrayPubSubType.class);
      registerPacketField(DynamicArrayPrimitivesNestedPubSubType.class);
      registerPacketField(NavSatStatusPubSubType.class);
      registerPacketField(HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType.class);
      registerPacketField(HumanoidBehaviorTypePacketPubSubType.class);
      registerPacketField(Vector3StampedPubSubType.class);
      registerPacketField(InteractiveMarkerUpdatePubSubType.class);
      registerPacketField(BlackFlyParameterPacketPubSubType.class);
      registerPacketField(GridCellsPubSubType.class);
      registerPacketField(HeightQuadTreeLeafMessagePubSubType.class);
      registerPacketField(Polygon2DMessagePubSubType.class);
      registerPacketField(WalkingControllerFailureStatusMessagePubSubType.class);
      registerPacketField(ParameterValuePubSubType.class);
      registerPacketField(FootstepDataMessagePubSubType.class);
      registerPacketField(FootstepPlanningToolboxOutputStatusPubSubType.class);
      registerPacketField(ColorRGBAPubSubType.class);
      registerPacketField(PelvisPoseErrorPacketPubSubType.class);
      registerPacketField(InteractiveMarkerPubSubType.class);
      registerPacketField(AtlasDesiredPumpPSIPacketPubSubType.class);
      registerPacketField(RequestStereoPointCloudMessagePubSubType.class);
      registerPacketField(Int64PubSubType.class);
      registerPacketField(ExoskeletonBehaviorStatePacketPubSubType.class);
      registerPacketField(Float64MultiArrayPubSubType.class);
      registerPacketField(HighLevelStateChangeStatusMessagePubSubType.class);
      registerPacketField(JoyFeedbackPubSubType.class);
      registerPacketField(BDIBehaviorCommandPacketPubSubType.class);
      registerPacketField(Float32MultiArrayPubSubType.class);
      registerPacketField(TF2ErrorPubSubType.class);
      registerPacketField(MapMetaDataPubSubType.class);
      registerPacketField(LaserScanPubSubType.class);
      registerPacketField(NavSatFixPubSubType.class);
      registerPacketField(BoundedArrayNestedPubSubType.class);
      registerPacketField(AtlasElectricMotorEnablePacketPubSubType.class);
      registerPacketField(Pose2DPubSubType.class);
      registerPacketField(SnapFootstepPacketPubSubType.class);
      registerPacketField(WalkingStatusMessagePubSubType.class);
      registerPacketField(MultisenseParameterPacketPubSubType.class);
      registerPacketField(PoseArrayPubSubType.class);
      registerPacketField(WeightMatrix3DMessagePubSubType.class);
      registerPacketField(FootstepPlanRequestPacketPubSubType.class);
      registerPacketField(ReachingManifoldMessagePubSubType.class);
      registerPacketField(BoundedArrayPrimitivesPubSubType.class);
      registerPacketField(PolygonPubSubType.class);
      registerPacketField(ParameterDescriptorPubSubType.class);
      registerPacketField(WallPosePacketPubSubType.class);
      registerPacketField(StopAllTrajectoryMessagePubSubType.class);
      registerPacketField(Int8MultiArrayPubSubType.class);
      registerPacketField(DesiredAccelerationsMessagePubSubType.class);
      registerPacketField(DoorLocationPacketPubSubType.class);
      registerPacketField(MultiDOFJointTrajectoryPointPubSubType.class);
      registerPacketField(ChannelFloat32PubSubType.class);
      registerPacketField(SCSListenerPacketPubSubType.class);
      registerPacketField(TransformStampedPubSubType.class);
      registerPacketField(AccelWithCovarianceStampedPubSubType.class);
      registerPacketField(DetectedFacesPacketPubSubType.class);
      registerPacketField(RobotConfigurationDataPubSubType.class);
      registerPacketField(MessageCollectionPubSubType.class);
      registerPacketField(MessageCollectionNotificationPubSubType.class);
      registerPacketField(GoHomeMessagePubSubType.class);
      registerPacketField(KinematicsToolboxOutputStatusPubSubType.class);
      registerPacketField(AccelWithCovariancePubSubType.class);
      registerPacketField(ManualHandControlPacketPubSubType.class);
      registerPacketField(PelvisTrajectoryMessagePubSubType.class);
      registerPacketField(Point32PubSubType.class);
      registerPacketField(MultiDOFJointTrajectoryPubSubType.class);
      registerPacketField(InvalidPacketNotificationPacketPubSubType.class);
      registerPacketField(CenterOfMassTrajectoryMessagePubSubType.class);
      registerPacketField(PointCloudPubSubType.class);
      registerPacketField(UIPositionCheckerPacketPubSubType.class);
      registerPacketField(WaypointBasedTrajectoryMessagePubSubType.class);
      registerPacketField(HeadHybridJointspaceTaskspaceTrajectoryMessagePubSubType.class);
      registerPacketField(PoseWithCovarianceStampedPubSubType.class);
      registerPacketField(ChestHybridJointspaceTaskspaceTrajectoryMessagePubSubType.class);
      registerPacketField(PilotAlarmPacketPubSubType.class);
      registerPacketField(PoseWithCovariancePubSubType.class);
      registerPacketField(VideoPacketPubSubType.class);
      registerPacketField(Int32PubSubType.class);
      registerPacketField(ByteMultiArrayPubSubType.class);
      registerPacketField(HandPowerCyclePacketPubSubType.class);
      registerPacketField(QuaternionPubSubType.class);
      registerPacketField(PelvisHeightTrajectoryMessagePubSubType.class);
      registerPacketField(JointStatePubSubType.class);
      registerPacketField(QuaternionStampedPubSubType.class);
      registerPacketField(SimulatedLidarScanPacketPubSubType.class);
      registerPacketField(PilotInterfacePacketPubSubType.class);
      registerPacketField(JointspaceTrajectoryMessagePubSubType.class);
      registerPacketField(QueueableMessagePubSubType.class);
      registerPacketField(IlluminancePubSubType.class);
      registerPacketField(NeckDesiredAccelerationsMessagePubSubType.class);
      registerPacketField(LoadBearingMessagePubSubType.class);
      registerPacketField(PilotInterfaceActionPacketPubSubType.class);
      registerPacketField(Int8PubSubType.class);
      registerPacketField(Float32PubSubType.class);
      registerPacketField(WalkToGoalBehaviorPacketPubSubType.class);
      registerPacketField(SpatialVectorMessagePubSubType.class);
      registerPacketField(UInt16PubSubType.class);
      registerPacketField(Float64PubSubType.class);
      registerPacketField(LocalizationPacketPubSubType.class);
      registerPacketField(KinematicsToolboxCenterOfMassMessagePubSubType.class);
      registerPacketField(BDIBehaviorStatusPacketPubSubType.class);
      registerPacketField(TwistWithCovariancePubSubType.class);
      registerPacketField(NestedPubSubType.class);
      registerPacketField(DynamicArrayNestedPubSubType.class);
      registerPacketField(PosePubSubType.class);
      registerPacketField(UInt64MultiArrayPubSubType.class);
      registerPacketField(HandDesiredConfigurationMessagePubSubType.class);
      registerPacketField(CompressedImagePubSubType.class);
      registerPacketField(HeightQuadTreeMessagePubSubType.class);
      registerPacketField(InertiaStampedPubSubType.class);
      registerPacketField(LidarScanMessagePubSubType.class);
      registerPacketField(PointPubSubType.class);
      registerPacketField(KinematicsToolboxRigidBodyMessagePubSubType.class);
      registerPacketField(PointCloudWorldPacketPubSubType.class);
      registerPacketField(WholeBodyTrajectoryMessagePubSubType.class);
      registerPacketField(MultiEchoLaserScanPubSubType.class);
      registerPacketField(LidarScanParametersMessagePubSubType.class);
      registerPacketField(RelativeHumidityPubSubType.class);
      registerPacketField(FrameInformationPubSubType.class);
      registerPacketField(HandTrajectoryMessagePubSubType.class);
      registerPacketField(ObjectDetectorResultPacketPubSubType.class);
      registerPacketField(StringPubSubType.class);
      registerPacketField(DurationPubSubType.class);
      registerPacketField(MarkerArrayPubSubType.class);
      registerPacketField(WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.class);
      registerPacketField(HighLevelStateMessagePubSubType.class);
      registerPacketField(UInt16MultiArrayPubSubType.class);
      registerPacketField(PelvisOrientationTrajectoryMessagePubSubType.class);
      registerPacketField(LaserEchoPubSubType.class);
      registerPacketField(HeatMapPacketPubSubType.class);
      registerPacketField(StatePubSubType.class);
      registerPacketField(DisparityImagePubSubType.class);
      registerPacketField(AtlasAuxiliaryRobotDataPubSubType.class);
      registerPacketField(TemperaturePubSubType.class);
      registerPacketField(IntraProcessMessagePubSubType.class);
      registerPacketField(HumanoidKinematicsToolboxConfigurationMessagePubSubType.class);
      registerPacketField(BatteryStatePubSubType.class);
      registerPacketField(SO3TrajectoryMessagePubSubType.class);
      registerPacketField(FootstepPathPlanPacketPubSubType.class);
      registerPacketField(NeckTrajectoryMessagePubSubType.class);
      registerPacketField(BytePubSubType.class);
      registerPacketField(DiagnosticArrayPubSubType.class);
      registerPacketField(ImagePubSubType.class);
      registerPacketField(SE3TrajectoryPointMessagePubSubType.class);
      registerPacketField(LocalizationPointMapPacketPubSubType.class);
      registerPacketField(JointTrajectoryPointPubSubType.class);
      registerPacketField(SimpleCoactiveBehaviorDataPacketPubSubType.class);
      registerPacketField(CapturabilityBasedStatusPubSubType.class);
      registerPacketField(CharPubSubType.class);
      registerPacketField(UInt32PubSubType.class);
      registerPacketField(ValveLocationPacketPubSubType.class);
      registerPacketField(GoalStatusArrayPubSubType.class);
      registerPacketField(MultiArrayLayoutPubSubType.class);
      registerPacketField(MomentumTrajectoryMessagePubSubType.class);
      registerPacketField(ArmTrajectoryMessagePubSubType.class);
      registerPacketField(ImuPubSubType.class);
      registerPacketField(OneDoFJointTrajectoryMessagePubSubType.class);
      registerPacketField(KeyValuePubSubType.class);
      registerPacketField(ManipulationAbortedStatusPubSubType.class);
      registerPacketField(LegCompliancePacketPubSubType.class);
      registerPacketField(PolygonStampedPubSubType.class);
      registerPacketField(TimeReferencePubSubType.class);
      registerPacketField(ParameterTypePubSubType.class);
      registerPacketField(FluidPressurePubSubType.class);
      registerPacketField(InteractiveMarkerInitPubSubType.class);
      registerPacketField(HeightQuadTreeToolboxRequestMessagePubSubType.class);
      registerPacketField(HandLoadBearingMessagePubSubType.class);
      registerPacketField(WholeBodyTrajectoryToolboxOutputStatusPubSubType.class);
      registerPacketField(JointTrajectoryPubSubType.class);
      registerPacketField(UInt64PubSubType.class);
      registerPacketField(BoundingBoxesPacketPubSubType.class);
      registerPacketField(RequestLidarScanMessagePubSubType.class);
      registerPacketField(PoseStampedPubSubType.class);
      registerPacketField(PointFieldPubSubType.class);
      registerPacketField(TwistWithCovarianceStampedPubSubType.class);
      registerPacketField(StateEstimatorModePacketPubSubType.class);
      registerPacketField(GoalIDPubSubType.class);
      registerPacketField(MultiDOFJointStatePubSubType.class);
      registerPacketField(MarkerPubSubType.class);
      registerPacketField(DiagnosticStatusPubSubType.class);
      registerPacketField(ImageMarkerPubSubType.class);
      registerPacketField(FootstepPlanningRequestPacketPubSubType.class);
      registerPacketField(TwistPubSubType.class);
      registerPacketField(TransitionEventPubSubType.class);
      registerPacketField(StampedPosePacketPubSubType.class);
      registerPacketField(Int32MultiArrayPubSubType.class);
      registerPacketField(TrajectoryPoint1DMessagePubSubType.class);
      registerPacketField(IntrinsicParametersMessagePubSubType.class);
      registerPacketField(SE3TrajectoryMessagePubSubType.class);
      registerPacketField(HandCollisionDetectedPacketPubSubType.class);
      registerPacketField(ParameterEventDescriptorsPubSubType.class);
      registerPacketField(TransformPubSubType.class);
      registerPacketField(DynamicArrayPrimitivesPubSubType.class);
      registerPacketField(AtlasWristSensorCalibrationRequestPacketPubSubType.class);
      registerPacketField(SetParametersResultPubSubType.class);
      registerPacketField(InteractiveMarkerFeedbackPubSubType.class);
      registerPacketField(Vector3PubSubType.class);
      registerPacketField(EmptyPubSubType.class);
      registerPacketField(UInt8MultiArrayPubSubType.class);
      registerPacketField(AutomaticManipulationAbortMessagePubSubType.class);
      registerPacketField(UInt8PubSubType.class);
      registerPacketField(Int16PubSubType.class);
      registerPacketField(PlanarRegionMessagePubSubType.class);
      registerPacketField(PauseWalkingMessagePubSubType.class);
      registerPacketField(TextToSpeechPacketPubSubType.class);
      registerPacketField(AdjustFootstepMessagePubSubType.class);
      registerPacketField(AccelPubSubType.class);
      registerPacketField(ParameterEventPubSubType.class);
      registerPacketField(TwistStampedPubSubType.class);
      registerPacketField(ClearDelayQueueMessagePubSubType.class);
      registerPacketField(ObjectWeightPacketPubSubType.class);
      registerPacketField(Int64MultiArrayPubSubType.class);
      registerPacketField(MeshTrianglePubSubType.class);
      registerPacketField(SolidPrimitivePubSubType.class);
      registerPacketField(MagneticFieldPubSubType.class);
      registerPacketField(GoalStatusPubSubType.class);
      registerPacketField(PrepareForLocomotionMessagePubSubType.class);
      registerPacketField(SelectionMatrix3DMessagePubSubType.class);
      registerPacketField(MeshPubSubType.class);
      registerPacketField(TransitionDescriptionPubSubType.class);
      registerPacketField(VehiclePosePacketPubSubType.class);
      registerPacketField(HeadTrajectoryMessagePubSubType.class);
      registerPacketField(WalkOverTerrainGoalPacketPubSubType.class);
      registerPacketField(PlanarRegionsListMessagePubSubType.class);
      registerPacketField(BehaviorControlModePacketPubSubType.class);
      registerPacketField(AccelStampedPubSubType.class);
      registerPacketField(ByteBufferPubSubType.class);
      registerPacketField(IMUPacketPubSubType.class);
      registerPacketField(ArmDesiredAccelerationsMessagePubSubType.class);
      registerPacketField(EuclideanTrajectoryMessagePubSubType.class);
      registerPacketField(ControllerCrashNotificationPacketPubSubType.class);
      registerPacketField(BoundingBox3DMessagePubSubType.class);
      registerPacketField(TransitionPubSubType.class);
      registerPacketField(InteractiveMarkerPosePubSubType.class);
      registerPacketField(DrillDetectionPacketPubSubType.class);

      registerPacketClass(Packet.class);

      registerPacketClass(MessageCollection.class);
      registerPacketClass(MessageCollectionNotification.class);

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
      registerPacketField(SpineTrajectoryMessage.class);
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
