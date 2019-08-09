package us.ihmc.humanoidRobotics.communication.packets;

import static us.ihmc.euclid.tools.EuclidCoreTools.zeroVector3D;

import java.awt.image.BufferedImage;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import boofcv.struct.calib.IntrinsicParameters;
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
import controller_msgs.msg.dds.BDIBehaviorStatusPacket;
import controller_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.BehaviorControlModeResponsePacket;
import controller_msgs.msg.dds.BehaviorStatusPacket;
import controller_msgs.msg.dds.BlackFlyParameterPacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.ClearDelayQueueMessage;
import controller_msgs.msg.dds.DesiredAccelerationsMessage;
import controller_msgs.msg.dds.DetectedObjectPacket;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.EuclideanTrajectoryMessage;
import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.FisheyePacket;
import controller_msgs.msg.dds.FootLoadBearingMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPathPlanPacket;
import controller_msgs.msg.dds.FootstepPlanRequestPacket;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandCollisionDetectedPacket;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.HandPowerCyclePacket;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.IntrinsicParametersMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.LegCompliancePacket;
import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.LocalizationPointMapPacket;
import controller_msgs.msg.dds.LocalizationStatusPacket;
import controller_msgs.msg.dds.ManualHandControlPacket;
import controller_msgs.msg.dds.MultisenseParameterPacket;
import controller_msgs.msg.dds.NeckDesiredAccelerationsMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.ObjectWeightPacket;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.PlanOffsetStatus;
import controller_msgs.msg.dds.PointCloudWorldPacket;
import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import controller_msgs.msg.dds.QueueableMessage;
import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.SE3TrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import controller_msgs.msg.dds.SnapFootstepPacket;
import controller_msgs.msg.dds.SpineDesiredAccelerationsMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StateEstimatorModePacket;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.ValveLocationPacket;
import controller_msgs.msg.dds.VehiclePosePacket;
import controller_msgs.msg.dds.VideoPacket;
import controller_msgs.msg.dds.WalkOverTerrainGoalPacket;
import controller_msgs.msg.dds.WalkToGoalBehaviorPacket;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import controller_msgs.msg.dds.WallPosePacket;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalAction;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class HumanoidMessageTools
{
   public static final int CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES = 8;

   private HumanoidMessageTools()
   {
   }

   public static BlackFlyParameterPacket createBlackFlyParameterPacket(boolean fromUI, double gain, double exposure, double frameRate, double shutter,
                                                                       boolean autoExposure, boolean autoGain, boolean autoShutter, RobotSide side)
   {
      BlackFlyParameterPacket message = new BlackFlyParameterPacket();
      message.setFromUi(fromUI);
      message.setExposure(exposure);
      message.setShutter(shutter);
      message.setGain(gain);
      message.setFrameRate(frameRate);
      message.setAutoExposure(autoExposure);
      message.setAutoGain(autoGain);
      message.setAutoShutter(autoShutter);
      message.setRobotSide(side.toByte());
      return message;
   }

   public static DesiredAccelerationsMessage createDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      DesiredAccelerationsMessage message = new DesiredAccelerationsMessage();
      message.getDesiredJointAccelerations().add(desiredJointAccelerations);
      return message;
   }

   public static NeckDesiredAccelerationsMessage createNeckDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      NeckDesiredAccelerationsMessage message = new NeckDesiredAccelerationsMessage();
      message.getDesiredAccelerations().set(HumanoidMessageTools.createDesiredAccelerationsMessage(desiredJointAccelerations));
      return message;
   }

   public static ChestHybridJointspaceTaskspaceTrajectoryMessage createChestHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                       JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ChestHybridJointspaceTaskspaceTrajectoryMessage message = new ChestHybridJointspaceTaskspaceTrajectoryMessage();
      message.getTaskspaceTrajectoryMessage().set(taskspaceTrajectoryMessage);
      message.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
      return message;
   }

   public static HeadHybridJointspaceTaskspaceTrajectoryMessage createHeadHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                     JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      HeadHybridJointspaceTaskspaceTrajectoryMessage message = new HeadHybridJointspaceTaskspaceTrajectoryMessage();
      message.getTaskspaceTrajectoryMessage().set(taskspaceTrajectoryMessage);
      message.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
      return message;
   }

   public static HandHybridJointspaceTaskspaceTrajectoryMessage createHandHybridJointspaceTaskspaceTrajectoryMessage(RobotSide robotSide,
                                                                                                                     SE3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                     JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      HandHybridJointspaceTaskspaceTrajectoryMessage message = new HandHybridJointspaceTaskspaceTrajectoryMessage();
      message.setRobotSide(robotSide.toByte());
      message.getTaskspaceTrajectoryMessage().set(taskspaceTrajectoryMessage);
      message.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
      return message;
   }

   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.getJointspaceTrajectory().set(new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide             is used to define which arm is performing the trajectory.
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of arm joints.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      return createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions, null, null);
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights. Set
    * the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide             is used to define which arm is performing the trajectory.
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of arm joints.
    * @param weights               the qp weights for the joint accelerations. If any index is set to
    *                              NaN, that joint will use the controller default weight
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      return createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions, null, weights);
   }

   /**
    * Use this constructor to go straight to the given end points with final velocity using the
    * specified qp weights. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide              is used to define which arm is performing the trajectory.
    * @param trajectoryTime         how long it takes to reach the desired pose.
    * @param desiredJointPositions  desired joint positions. The array length should be equal to the
    *                               number of arm joints.
    * @param desiredJointVelocities desired final joint velocities. The array length should be equal to
    *                               the number of arm joints. Can be {@code null}.
    * @param weights                the qp weights for the joint accelerations. If any index is set to
    *                               NaN, that joint will use the controller default weight. Can be
    *                               {@code null}.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions,
                                                                 double[] desiredJointVelocities, double[] weights)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.getJointspaceTrajectory().set(createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, desiredJointVelocities, weights));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide                     is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.getJointspaceTrajectory().set(createJointspaceTrajectoryMessage(jointTrajectory1DListMessages));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards. Set
    * the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide is used to define which arm is performing the trajectory.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(trajectoryMessage);
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the
    * base for the control. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide          is used to define which hand is performing the trajectory.
    * @param trajectoryTime     how long it takes to reach the desired pose.
    * @param desiredPosition    desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameId)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrameId));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the
    * base for the control. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide          is used to define which hand is performing the trajectory.
    * @param trajectoryTime     how long it takes to reach the desired pose.
    * @param desiredPosition    desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Pose3DReadOnly desiredPose,
                                                                   ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPose, trajectoryReferenceFrame));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Pose3DReadOnly desiredPose,
                                                                   SpatialVectorReadOnly desiredVelocity, ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPose, desiredVelocity, trajectoryReferenceFrame));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static BehaviorStatusPacket createBehaviorStatusPacket(CurrentBehaviorStatus requestedControl)
   {
      BehaviorStatusPacket message = new BehaviorStatusPacket();
      message.setCurrentBehaviorStatus(requestedControl.toByte());
      return message;
   }

   public static LegCompliancePacket createLegCompliancePacket(float[] maxVelocityDeltas, RobotSide robotSide)
   {
      LegCompliancePacket message = new LegCompliancePacket();
      message.getMaxVelocityDeltas().add(maxVelocityDeltas);
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static SnapFootstepPacket createSnapFootstepPacket(List<FootstepDataMessage> data, int[] footstepOrder, byte[] flag)
   {
      SnapFootstepPacket message = new SnapFootstepPacket();
      MessageTools.copyData(data, message.getFootstepData());
      message.getFootstepOrder().add(footstepOrder);
      message.getFlag().add(flag);
      return message;
   }

   public static BehaviorControlModePacket createBehaviorControlModePacket(BehaviorControlModeEnum requestedControl)
   {
      BehaviorControlModePacket message = new BehaviorControlModePacket();
      message.setBehaviorControlModeEnumRequest(requestedControl.toByte());
      return message;
   }

   /**
    * Create a message to request one end-effector to switch to load bearing. Set the id of the message
    * to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide refers to the side of the end-effector if necessary.
    */
   public static FootLoadBearingMessage createFootLoadBearingMessage(RobotSide robotSide, LoadBearingRequest request)
   {
      FootLoadBearingMessage message = new FootLoadBearingMessage();
      message.setRobotSide(robotSide.toByte());
      message.setLoadBearingRequest(request.toByte());
      return message;
   }

   // joint values should be in the range [0,1]
   public static ManualHandControlPacket createManualHandControlPacket(RobotSide robotSide, double index, double middle, double thumb, double spread,
                                                                       int controlType)
   {
      ManualHandControlPacket message = new ManualHandControlPacket();
      message.setRobotSide(robotSide.toByte());
      message.setIndex(index);
      message.setMiddle(middle);
      message.setThumb(thumb);
      message.setSpread(spread);
      message.setControlType(controlType);
      return message;
   }

   public static MultisenseParameterPacket createMultisenseParameterPacket(boolean initialize, double gain, double motorSpeed, double dutyCycle,
                                                                           boolean ledEnable, boolean flashEnable, boolean autoExposure,
                                                                           boolean autoWhiteBalance)
   {
      MultisenseParameterPacket message = new MultisenseParameterPacket();
      message.setInitialize(initialize);
      message.setGain(gain);
      message.setFlashEnable(flashEnable);
      message.setMotorSpeed(motorSpeed);
      message.setLedEnable(ledEnable);
      message.setDutyCycle(dutyCycle);
      message.setAutoExposure(autoExposure);
      message.setAutoWhiteBalance(autoWhiteBalance);
      return message;
   }

   public static DoorLocationPacket createDoorLocationPacket(RigidBodyTransform doorTransformToWorld)
   {
      return createDoorLocationPacket(new Pose3D(doorTransformToWorld));
   }

   public static DoorLocationPacket createDoorLocationPacket(Pose3D doorTransformToWorld)
   {
      DoorLocationPacket message = new DoorLocationPacket();
      message.getDoorTransformToWorld().set(doorTransformToWorld);
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket(Point3D position, Quaternion orientation)
   {
      VehiclePosePacket message = new VehiclePosePacket();
      message.getPosition().set(position);
      message.getOrientation().set(orientation);
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket(RigidBodyTransform transformFromVehicleToWorld)
   {
      VehiclePosePacket message = new VehiclePosePacket();
      message.getOrientation().set(transformFromVehicleToWorld.getRotation());
      message.getPosition().set(transformFromVehicleToWorld.getTranslation());
      return message;
   }

   public static HighLevelStateChangeStatusMessage createHighLevelStateChangeStatusMessage(HighLevelControllerName initialState,
                                                                                           HighLevelControllerName endState)
   {
      HighLevelStateChangeStatusMessage message = new HighLevelStateChangeStatusMessage();
      message.setInitialHighLevelControllerName(initialState == null ? -1 : initialState.toByte());
      message.setEndHighLevelControllerName(endState == null ? -1 : endState.toByte());
      return message;
   }

   /**
    * To set disable exploration on this rigid body.
    */
   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBodyBasics rigidBody)
   {
      ConfigurationSpaceName[] configurations = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.YAW,
            ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL};
      double[] regionAmplitude = new double[] {0, 0, 0, 0, 0, 0};

      return createRigidBodyExplorationConfigurationMessage(rigidBody, configurations, regionAmplitude);
   }

   /**
    * To set enable exploration on this rigid body with following order of ConfigurationSpaceName.
    */
   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBodyBasics rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore)
   {
      return createRigidBodyExplorationConfigurationMessage(rigidBody,
                                                            degreesOfFreedomToExplore,
                                                            WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationAmplitudeArray(degreesOfFreedomToExplore));
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBodyBasics rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                                                                         double[] explorationRangeAmplitudes)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      if (degreesOfFreedomToExplore.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      message.setRigidBodyHashCode(rigidBody.hashCode());
      byte[] degreesOfFreedomToExplore1 = ConfigurationSpaceName.toBytes(degreesOfFreedomToExplore);
      if (degreesOfFreedomToExplore1.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore1.length
               + ", explorationRangeLowerLimits.length = ");

      message.getConfigurationSpaceNamesToExplore().reset();
      message.getExplorationRangeUpperLimits().reset();
      message.getExplorationRangeLowerLimits().reset();

      message.getConfigurationSpaceNamesToExplore().add(degreesOfFreedomToExplore1);

      for (int i = 0; i < degreesOfFreedomToExplore1.length; i++)
      {
         message.getExplorationRangeUpperLimits().add(explorationRangeAmplitudes[i]);
         message.getExplorationRangeLowerLimits().add(-explorationRangeAmplitudes[i]);
      }

      return message;
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBodyBasics rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                                                                         double[] explorationRangeUpperLimits,
                                                                                                         double[] explorationRangeLowerLimits)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      if (degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      message.setRigidBodyHashCode(rigidBody.hashCode());
      byte[] degreesOfFreedomToExplore1 = ConfigurationSpaceName.toBytes(degreesOfFreedomToExplore);
      if (degreesOfFreedomToExplore1.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore1.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore1.length
               + ", explorationRangeLowerLimits.length = ");

      message.getConfigurationSpaceNamesToExplore().reset();
      message.getExplorationRangeUpperLimits().reset();
      message.getExplorationRangeLowerLimits().reset();

      message.getConfigurationSpaceNamesToExplore().add(degreesOfFreedomToExplore1);
      message.getExplorationRangeUpperLimits().add(explorationRangeUpperLimits);
      message.getExplorationRangeLowerLimits().add(explorationRangeLowerLimits);

      return message;
   }

   public static FootstepPathPlanPacket createFootstepPathPlanPacket(boolean goalsValid, FootstepDataMessage start, List<FootstepDataMessage> originalGoals,
                                                                     List<FootstepDataMessage> ADStarPathPlan, List<Boolean> footstepUnknown,
                                                                     double subOptimality, double cost)
   {
      FootstepPathPlanPacket message = new FootstepPathPlanPacket();
      message.setGoalsValid(goalsValid);
      message.getStart().set(start);
      MessageTools.copyData(originalGoals, message.getOriginalGoals());
      MessageTools.copyData(ADStarPathPlan, message.getPathPlan());
      footstepUnknown.stream().forEach(message.getFootstepUnknown()::add);
      message.setSubOptimality(subOptimality);
      message.setPathCost(cost);
      return message;
   }

   public static ObjectWeightPacket createObjectWeightPacket(RobotSide robotSide, double weight)
   {
      ObjectWeightPacket message = new ObjectWeightPacket();
      message.setRobotSide(robotSide.toByte());
      message.setWeight(weight);
      return message;
   }

   public static HandPowerCyclePacket createHandPowerCyclePacket(RobotSide robotSide)
   {
      HandPowerCyclePacket message = new HandPowerCyclePacket();
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBodyBasics endEffector, double[] waypointTimes, Pose3D[] waypoints)
   {
      return createWaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints);
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBodyBasics endEffector, double[] waypointTimes, Pose3D[] waypoints,
                                                                                     SelectionMatrix6D selectionMatrix)
   {
      WaypointBasedTrajectoryMessage message = new WaypointBasedTrajectoryMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      if (waypointTimes.length != waypoints.length)
         throw new RuntimeException("Inconsistent array lengths.");

      message.getWaypointTimes().reset();
      message.getWaypointTimes().add(waypointTimes);
      MessageTools.copyData(waypoints, message.getWaypoints());
      if (selectionMatrix != null)
      {
         message.getAngularSelectionMatrix().setSelectionFrameId(MessageTools.toFrameId(selectionMatrix.getAngularSelectionFrame()));
         message.getAngularSelectionMatrix().setXSelected(selectionMatrix.isAngularXSelected());
         message.getAngularSelectionMatrix().setYSelected(selectionMatrix.isAngularYSelected());
         message.getAngularSelectionMatrix().setZSelected(selectionMatrix.isAngularZSelected());
         message.getLinearSelectionMatrix().setSelectionFrameId(MessageTools.toFrameId(selectionMatrix.getLinearSelectionFrame()));
         message.getLinearSelectionMatrix().setXSelected(selectionMatrix.isLinearXSelected());
         message.getLinearSelectionMatrix().setYSelected(selectionMatrix.isLinearYSelected());
         message.getLinearSelectionMatrix().setZSelected(selectionMatrix.isLinearZSelected());
      }
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime     how long it takes to reach the desired pose.
    * @param desiredPosition    desired pelvis position expressed in world frame.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                       QuaternionReadOnly desiredOrientation)
   {
      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, ReferenceFrame.getWorldFrame()));
      return message;
   }

   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(double trajectoryTime, Pose3DReadOnly desiredPose)
   {
      return createPelvisTrajectoryMessage(trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation());
   }

   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(double trajectoryTime, Pose3DReadOnly desiredPose, SpatialVectorReadOnly desiredVelocity)
   {
      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPose, desiredVelocity, ReferenceFrame.getWorldFrame()));
      return message;
   }

   public static PelvisPoseErrorPacket createPelvisPoseErrorPacket(float residualError, float totalError, boolean hasMapBeenReset)
   {
      PelvisPoseErrorPacket message = new PelvisPoseErrorPacket();
      message.setResidualError(residualError);
      message.setTotalError(totalError);
      message.setHasMapBeenReset(hasMapBeenReset);
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                                   List<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      AdjustFootstepMessage message = new AdjustFootstepMessage();
      message.setRobotSide(robotSide.toByte());
      message.getLocation().set(location);
      message.getOrientation().set(orientation);
      if (predictedContactPoints != null)
         MessageTools.copyData(predictedContactPoints.stream().map(Point3D::new).collect(Collectors.toList()), message.getPredictedContactPoints2d());
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType,
                                                                   double swingHeight)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                                   List<Point2D> predictedContactPoints)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, null);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(Footstep footstep)
   {
      AdjustFootstepMessage message = new AdjustFootstepMessage();
      message.setRobotSide(footstep.getRobotSide().toByte());

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      message.getLocation().set(location);
      message.getOrientation().set(orientation);
      MessageTools.copyData(footstep.getPredictedContactPoints().stream().map(Point3D::new).collect(Collectors.toList()),
                            message.getPredictedContactPoints2d());
      return message;
   }

   public static NeckTrajectoryMessage createNeckTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of joints.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      return createNeckTrajectoryMessage(trajectoryTime, desiredJointPositions, null, null);
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of joints.
    * @param weights               the qp weights for the joint accelerations. If any index is set to
    *                              NaN, that joint will use the controller default weight
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      return createNeckTrajectoryMessage(trajectoryTime, desiredJointPositions, null, weights);
   }

   /**
    * Use this constructor to go straight to the given end points with final velocity using the given
    * weights. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of joints. Can be {@code null}.
    * @param weights               the qp weights for the joint accelerations. If any index is set to
    *                              NaN, that joint will use the controller default weight. Can be
    *                              {@code null}.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] desiredJointVelocities,
                                                                   double[] weights)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.getJointspaceTrajectory().set(createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, desiredJointVelocities, weights));
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.getJointspaceTrajectory().set(createJointspaceTrajectoryMessage(jointTrajectory1DListMessages));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation towards the given endpoint. Set the id of
    * the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime     how long it takes to reach the desired pose.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, ReferenceFrame.getWorldFrame()));
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                                             ReferenceFrame trajectoryFrame)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryFrame));
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                                             Vector3DReadOnly desiredAngularVelocity,
                                                                                             ReferenceFrame trajectoryFrame)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame));
      return message;
   }

   public static WholeBodyTrajectoryToolboxMessage createWholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                                                                           List<WaypointBasedTrajectoryMessage> endEffectorTrajectories,
                                                                                           List<ReachingManifoldMessage> reachingManifolds,
                                                                                           List<RigidBodyExplorationConfigurationMessage> explorationConfigurations)
   {
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage();
      message.getConfiguration().set(configuration);
      MessageTools.copyData(endEffectorTrajectories, message.getEndEffectorTrajectories());
      MessageTools.copyData(reachingManifolds, message.getReachingManifolds());
      MessageTools.copyData(explorationConfigurations, message.getExplorationConfigurations());
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket(BDIRobotBehavior atlasRobotBehavior)
   {
      BDIBehaviorCommandPacket message = new BDIBehaviorCommandPacket();
      message.setAtlasBdiRobotBehavior(atlasRobotBehavior.toByte());
      return message;
   }

   public static AtlasElectricMotorEnablePacket createAtlasElectricMotorEnablePacket(AtlasElectricMotorPacketEnum motorEnableEnum, boolean enable)
   {
      AtlasElectricMotorEnablePacket message = new AtlasElectricMotorEnablePacket();
      message.setAtlasElectricMotorPacketEnumEnable(motorEnableEnum.toByte());
      message.setEnable(enable);
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(SO3TrajectoryMessage so3Trajectory)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory().set(new SO3TrajectoryMessage(so3Trajectory));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    *
    * @param trajectoryTime     how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed in World.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     long trajectoryReferenceFrameID)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryReferenceFrameID));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    *
    * @param trajectoryTime     how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed the supplied frame.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     ReferenceFrame trajectoryFrame)
   {
      return createChestTrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryFrame);
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    *
    * @param trajectoryTime         how long it takes to reach the desired orientation.
    * @param desiredOrientation     desired chest orientation expressed the supplied frame.
    * @param desiredAngularVelocity desired angular velocity at the end of the trajectory.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     Vector3DReadOnly desiredAngularVelocity, ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame));
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly quaternion, ReferenceFrame dataFrame,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage(trajectoryTime, quaternion, trajectoryFrame);
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataFrame));
      return message;
   }

   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame dataFrame,
                                                                   ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryFrame));
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataFrame));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime     how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryFrame));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime     how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                   long trajectoryReferenceFrameId)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryReferenceFrameId));
      return message;
   }

   /**
    * set a single point
    *
    * @param trajectoryTime             the duration of the trajectory
    * @param desiredPosition            the desired end position
    * @param trajectoryReferenceFrameId the frame id the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             long trajectoryReferenceFrameId)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, zeroVector3D, trajectoryReferenceFrameId);
   }

   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             Vector3DReadOnly desiredLinearVelocity, long trajectoryReferenceFrameId)
   {
      EuclideanTrajectoryMessage message = new EuclideanTrajectoryMessage();
      message.getTaskspaceTrajectoryPoints().add().set(createEuclideanTrajectoryPointMessage(trajectoryTime, desiredPosition, desiredLinearVelocity));
      message.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   /**
    * set a single point
    *
    * @param trajectoryTime           the duration of the trajectory
    * @param desiredPosition          the desired end position
    * @param trajectoryReferenceFrame the frame the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             ReferenceFrame trajectoryReferenceFrame)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, zeroVector3D, trajectoryReferenceFrame);
   }

   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             Vector3DReadOnly desiredLinearVelocity, ReferenceFrame trajectoryReferenceFrame)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, desiredLinearVelocity, trajectoryReferenceFrame.hashCode());
   }

   public static LocalizationPacket createLocalizationPacket(boolean reset, boolean toggle)
   {
      LocalizationPacket message = new LocalizationPacket();
      message.setReset(reset);
      message.setToggle(toggle);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime           how long it takes to reach the desired height.
    * @param desiredHeight            desired pelvis height expressed in data frame
    * @param trajectoryReferenceFrame the frame in which the height will be executed
    * @param dataReferenceFrame       the frame the desiredHeight is expressed in, the height will be
    *                                 changed to the trajectory frame on the controller side
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight,
                                                                                   ReferenceFrame trajectoryReferenceFrame, ReferenceFrame dataReferenceFrame)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                        new Point3D(0.0, 0.0, desiredHeight),
                                                                        trajectoryReferenceFrame.hashCode()));
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataReferenceFrame));
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. The trajectory and data frame are set
    * to world frame Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight  desired pelvis height expressed in world frame.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight), ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. The trajectory and data frame are set
    * to world frame Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime    how long it takes to reach the desired height.
    * @param desiredHeight     desired pelvis height expressed in world frame.
    * @param desiredHeightRate the desired rate of change of height when the desired height is reached.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight, double desiredHeightRate)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                        new Point3D(0.0, 0.0, desiredHeight),
                                                                        new Vector3D(0.0, 0.0, desiredHeightRate),
                                                                        ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint} for
    * each trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage()
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.setFootstepStatus(status.toByte());
      message.setFootstepIndex(footstepIndex);
      message.getDesiredFootPositionInWorld().setToNaN();
      message.getDesiredFootOrientationInWorld().setToNaN();
      message.getActualFootPositionInWorld().setToNaN();
      message.getActualFootOrientationInWorld().setToNaN();
      message.setRobotSide((byte) 255);
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, RobotSide robotSide)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.setFootstepStatus(status.toByte());
      message.setFootstepIndex(footstepIndex);
      message.getDesiredFootPositionInWorld().setToNaN();
      message.getDesiredFootOrientationInWorld().setToNaN();
      message.getActualFootPositionInWorld().setToNaN();
      message.getActualFootOrientationInWorld().setToNaN();
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.setFootstepStatus(status.toByte());
      message.setFootstepIndex(footstepIndex);
      message.getDesiredFootPositionInWorld().setToNaN();
      message.getDesiredFootOrientationInWorld().setToNaN();
      message.getActualFootPositionInWorld().set(actualFootPositionInWorld);
      message.getActualFootOrientationInWorld().set(actualFootOrientationInWorld);

      message.setRobotSide((byte) 255);
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.setFootstepStatus(status.toByte());
      message.setFootstepIndex(footstepIndex);
      message.getDesiredFootPositionInWorld().setToNaN();
      message.getDesiredFootOrientationInWorld().setToNaN();
      message.getActualFootPositionInWorld().set(actualFootPositionInWorld);
      message.getActualFootOrientationInWorld().set(actualFootOrientationInWorld);
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D desiredFootPositionInWorld,
                                                            Quaternion desiredFootOrientationInWorld, Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.setFootstepStatus(status.toByte());
      message.setFootstepIndex(footstepIndex);
      message.getDesiredFootPositionInWorld().set(desiredFootPositionInWorld);
      message.getDesiredFootOrientationInWorld().set(desiredFootOrientationInWorld);
      message.getActualFootPositionInWorld().set(actualFootPositionInWorld);
      message.getActualFootOrientationInWorld().set(actualFootOrientationInWorld);
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredAngularVelocity, ReferenceFrame trajectoryFrame)
   {
      return createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame.hashCode());
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredAngularVelocity, long trajectoryReferenceFrameId)
   {
      SO3TrajectoryMessage message = new SO3TrajectoryMessage();
      message.getTaskspaceTrajectoryPoints().add().set(createSO3TrajectoryPointMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity));
      message.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   public static HighLevelStateMessage createHighLevelStateMessage(HighLevelControllerName highLevelControllerName)
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(highLevelControllerName.toByte());
      return message;
   }

   public static WallPosePacket createWallPosePacket(WallPosePacket other)
   {
      WallPosePacket message = new WallPosePacket();
      message.set(other);
      return message;
   }

   public static WallPosePacket createWallPosePacket(double cuttingRadius, Tuple3DReadOnly centerPosition, QuaternionReadOnly centerOrientation)
   {
      WallPosePacket message = new WallPosePacket();
      message.setCuttingRadius(cuttingRadius);
      message.getCenterPosition().set(centerPosition);
      message.getCenterOrientation().set(centerOrientation);
      return message;
   }

   public static WallPosePacket createWallPosePacket(double cuttingRadius, Tuple3DReadOnly centerPosition, RotationMatrixReadOnly rotationMatrix)
   {
      WallPosePacket message = new WallPosePacket();
      message.setCuttingRadius(cuttingRadius);
      message.getCenterPosition().set(centerPosition);
      message.getCenterOrientation().set(new Quaternion(rotationMatrix));
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket(FootstepPlanRequestType requestType, FootstepDataMessage startFootstep,
                                                                           double thetaStart, List<FootstepDataMessage> goals)
   {
      FootstepPlanRequestPacket message = new FootstepPlanRequestPacket();
      message.setFootstepPlanRequestType(requestType.toByte());
      message.getStartFootstep().set(startFootstep);
      message.setThetaStart(thetaStart);
      MessageTools.copyData(goals, message.getGoals());
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket(FootstepPlanRequestType requestType, FootstepDataMessage startFootstep,
                                                                           double thetaStart, List<FootstepDataMessage> goals, double maxSuboptimality)
   {
      FootstepPlanRequestPacket message = new FootstepPlanRequestPacket();
      message.setFootstepPlanRequestType(requestType.toByte());
      message.getStartFootstep().set(startFootstep);
      message.setThetaStart(thetaStart);
      MessageTools.copyData(goals, message.getGoals());
      message.setMaxSubOptimality(maxSuboptimality);
      return message;
   }

   public static HandJointAnglePacket createHandJointAnglePacket(RobotSide robotSide, boolean connected, boolean calibrated, double[] jointAngles)
   {
      HandJointAnglePacket message = new HandJointAnglePacket();
      message.setRobotSide(robotSide == null ? -1 : robotSide.toByte());
      message.getJointAngles().add(jointAngles);
      message.setConnected(connected);
      message.setCalibrated(calibrated);
      return message;
   }

   public static HandCollisionDetectedPacket createHandCollisionDetectedPacket(RobotSide robotSide, int collisionSeverityLevelZeroToThree)
   {
      HandCollisionDetectedPacket message = new HandCollisionDetectedPacket();
      message.setRobotSide(robotSide.toByte());
      message.setCollisionSeverityLevelOneToThree(MathTools.clamp(collisionSeverityLevelZeroToThree, 1, 3));
      return message;
   }

   public static AtlasLowLevelControlModeMessage createAtlasLowLevelControlModeMessage(AtlasLowLevelControlMode request)
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(request.toByte());
      return message;
   }

   public static HandLoadBearingMessage createHandLoadBearingMessage(RobotSide robotSide)
   {
      HandLoadBearingMessage message = new HandLoadBearingMessage();
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static BehaviorControlModeResponsePacket createBehaviorControlModeResponsePacket(BehaviorControlModeEnum requestedControl)
   {
      BehaviorControlModeResponsePacket message = new BehaviorControlModeResponsePacket();
      message.setBehaviorControlModeEnumRequest(requestedControl.toByte());
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage(OneDoFTrajectoryPointBasics trajectoryPoint)
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.setTime(trajectoryPoint.getTime());
      message.setPosition(trajectoryPoint.getPosition());
      message.setVelocity(trajectoryPoint.getVelocity());
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage(double time, double position, double velocity)
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.setTime(time);
      message.setPosition(position);
      message.setVelocity(velocity);
      return message;
   }

   public static StateEstimatorModePacket createStateEstimatorModePacket(StateEstimatorMode requestedOperatingMode)
   {
      StateEstimatorModePacket message = new StateEstimatorModePacket();
      message.setRequestedStateEstimatorMode(requestedOperatingMode.toByte());
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(FullHumanoidRobotModel fullRobotModel)
   {
      return MessageTools.createKinematicsToolboxOutputStatus(fullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel));
   }

   public static HumanoidBehaviorTypePacket createHumanoidBehaviorTypePacket(HumanoidBehaviorType behaviorType)
   {
      HumanoidBehaviorTypePacket message = new HumanoidBehaviorTypePacket();
      message.setHumanoidBehaviorType(behaviorType.toByte());
      return message;
   }

   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double finalTransferDuration)
   {
      return createFootstepDataListMessage(footstepDataList, 0.0, 0.0, finalTransferDuration, ExecutionMode.OVERRIDE);
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double defaultSwingDuration,
                                                                       double defaultTransferDuration, ExecutionMode executionMode)
   {
      return createFootstepDataListMessage(footstepDataList, defaultSwingDuration, defaultTransferDuration, defaultTransferDuration, executionMode);
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double defaultSwingDuration,
                                                                       double defaultTransferDuration, double finalTransferDuration,
                                                                       ExecutionMode executionMode)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      MessageTools.copyData(footstepDataList, message.getFootstepDataList());
      message.setDefaultSwingDuration(defaultSwingDuration);
      message.setDefaultTransferDuration(defaultTransferDuration);
      message.setFinalTransferDuration(finalTransferDuration);
      return message;
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to
    * OVERRIDE
    *
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    */
   public static FootstepDataListMessage createFootstepDataListMessage(double defaultSwingDuration, double defaultTransferDuration)
   {
      return createFootstepDataListMessage(defaultSwingDuration, defaultTransferDuration, defaultTransferDuration);
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. Set execution mode to
    * OVERRIDE
    *
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    */
   public static FootstepDataListMessage createFootstepDataListMessage(double defaultSwingDuration, double defaultTransferDuration,
                                                                       double finalTransferDuration)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      message.setDefaultSwingDuration(defaultSwingDuration);
      message.setDefaultTransferDuration(defaultTransferDuration);
      message.setFinalTransferDuration(finalTransferDuration);
      return message;
   }

   /**
    * Creates a message with the desired grasp to be performed. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide                refers to which hand will perform the grasp.
    * @param handDesiredConfiguration refers to the desired grasp.
    */
   public static HandDesiredConfigurationMessage createHandDesiredConfigurationMessage(RobotSide robotSide, HandConfiguration handDesiredConfiguration)
   {
      HandDesiredConfigurationMessage message = new HandDesiredConfigurationMessage();
      message.setRobotSide(robotSide.toByte());
      message.setDesiredHandConfiguration(handDesiredConfiguration.toByte());
      return message;
   }

   public static FisheyePacket createFisheyePacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position,
                                                   QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      FisheyePacket message = new FisheyePacket();
      message.getVideoPacket().set(createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters));
      return message;
   }

   public static VideoPacket createVideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation,
                                               IntrinsicParameters intrinsicParameters)
   {
      VideoPacket message = new VideoPacket();
      message.setVideoSource(videoSource.toByte());
      message.setTimestamp(timeStamp);
      message.getData().add(data);
      message.getPosition().set(position);
      message.getOrientation().set(orientation);
      message.getIntrinsicParameters().set(toIntrinsicParametersMessage(intrinsicParameters));
      return message;
   }

   public static LocalVideoPacket createLocalVideoPacket(long timeStamp, BufferedImage image, IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket message = new LocalVideoPacket();
      message.timeStamp = timeStamp;
      message.image = image;
      message.intrinsicParameters = toIntrinsicParametersMessage(intrinsicParameters);
      return message;
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param pause
    */
   public static PauseWalkingMessage createPauseWalkingMessage(boolean pause)
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(pause);
      return message;
   }

   public static ReachingManifoldMessage createReachingManifoldMessage(RigidBodyBasics rigidBody)
   {
      ReachingManifoldMessage message = new ReachingManifoldMessage();
      message.setEndEffectorHashCode(rigidBody.hashCode());
      return message;
   }

   public static WholeBodyTrajectoryToolboxConfigurationMessage createWholeBodyTrajectoryToolboxConfigurationMessage(int numberOfInitialGuesses)
   {
      return createWholeBodyTrajectoryToolboxConfigurationMessage(numberOfInitialGuesses, -1);
   }

   public static WholeBodyTrajectoryToolboxConfigurationMessage createWholeBodyTrajectoryToolboxConfigurationMessage(int numberOfInitialGuesses,
                                                                                                                     int maximumExpansionSize)
   {
      WholeBodyTrajectoryToolboxConfigurationMessage message = new WholeBodyTrajectoryToolboxConfigurationMessage();
      message.setNumberOfInitialGuesses(numberOfInitialGuesses);
      message.setMaximumExpansionSize(maximumExpansionSize);
      return message;
   }

   public static SO3TrajectoryPointMessage createSO3TrajectoryPointMessage(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      SO3TrajectoryPointMessage message = new SO3TrajectoryPointMessage();
      message.setTime(time);
      message.getOrientation().set(new Quaternion(orientation));
      message.getAngularVelocity().set(new Vector3D(angularVelocity));
      return message;
   }

   public static ArmDesiredAccelerationsMessage createArmDesiredAccelerationsMessage(RobotSide robotSide, double[] armDesiredJointAccelerations)
   {
      ArmDesiredAccelerationsMessage message = new ArmDesiredAccelerationsMessage();
      message.getDesiredAccelerations().set(HumanoidMessageTools.createDesiredAccelerationsMessage(armDesiredJointAccelerations));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Constructor that sets the desired accelerations in this message to the provided values
    *
    * @param desiredJointAccelerations
    */
   public static SpineDesiredAccelerationsMessage createSpineDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      SpineDesiredAccelerationsMessage message = new SpineDesiredAccelerationsMessage();
      message.getDesiredAccelerations().set(HumanoidMessageTools.createDesiredAccelerationsMessage(desiredJointAccelerations));
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of controlled joints.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
         message.getJointTrajectoryMessages().add().set(createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]));
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights. Set
    * the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of controlled joints.
    * @param weights               the qp weights for the joint accelerations
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
         oneDoFJointTrajectoryMessage.setWeight(weights[jointIndex]);
         message.getJointTrajectoryMessages().add().set(oneDoFJointTrajectoryMessage);
      }
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights. Set
    * the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime         how long it takes to reach the desired pose.
    * @param desiredJointPositions  desired joint positions. The array length should be equal to the
    *                               number of controlled joints.
    * @param desiredJointVelocities desired joint velocities. The array length should be equal to the
    *                               number of controlled joints. Can be {@code null}.
    * @param weights                the qp weights for the joint accelerations. The array length should
    *                               be equal to the number of controlled joints. Can be {@code null}.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions,
                                                                               double[] desiredJointVelocities, double[] weights)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage;
         if (desiredJointVelocities == null)
         {
            if (weights == null)
               oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
            else
               oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex], weights[jointIndex]);
         }
         else
         {
            if (weights == null)
               oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime,
                                                                                 desiredJointPositions[jointIndex],
                                                                                 desiredJointVelocities[jointIndex],
                                                                                 -1.0);
            else
               oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime,
                                                                                 desiredJointPositions[jointIndex],
                                                                                 desiredJointVelocities[jointIndex],
                                                                                 weights[jointIndex]);
         }
         message.getJointTrajectoryMessages().add().set(oneDoFJointTrajectoryMessage);
      }
      return message;
   }

   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double[] trajectoryTimes, double[] desiredJointPositions)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
         message.getJointTrajectoryMessages().add().set(createOneDoFJointTrajectoryMessage(trajectoryTimes[jointIndex], desiredJointPositions[jointIndex]));
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param oneDoFJointTrajectoryMessages joint trajectory points to be executed.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(OneDoFJointTrajectoryMessage[] oneDoFJointTrajectoryMessages)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      MessageTools.copyData(oneDoFJointTrajectoryMessages, message.getJointTrajectoryMessages());
      return message;
   }

   public static SimpleCoactiveBehaviorDataPacket createSimpleCoactiveBehaviorDataPacket(String key, double value)
   {
      SimpleCoactiveBehaviorDataPacket message = new SimpleCoactiveBehaviorDataPacket();
      message.setKey(key);
      message.setValue(value);
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket(boolean stop)
   {
      BDIBehaviorCommandPacket message = new BDIBehaviorCommandPacket();
      message.setStop(stop);
      return message;
   }

   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(OneDoFTrajectoryPointList trajectoryData)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      int numberOfPoints = trajectoryData.getNumberOfTrajectoryPoints();

      for (int i = 0; i < numberOfPoints; i++)
      {
         OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(i);
         message.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
      }
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point.
    *
    * @param trajectoryTime  how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition)
   {
      return createOneDoFJointTrajectoryMessage(trajectoryTime, desiredPosition, -1.0);
   }

   /**
    * Use this constructor to go straight to the given end point.
    *
    * @param trajectoryTime  how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    * @param weight          the weight for the qp
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition, double weight)
   {
      return createOneDoFJointTrajectoryMessage(trajectoryTime, desiredPosition, 0.0, weight);
   }

   /**
    * Use this constructor to go straight to the given end point and terminate at the given velocity.
    *
    * @param trajectoryTime  how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    * @param desiredVelocity desired final velocity.
    * @param weight          the weight for the qp
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition, double desiredVelocity,
                                                                                 double weight)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      message.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryTime, desiredPosition, desiredVelocity));
      message.setWeight(weight);
      return message;
   }

   public static AtlasDesiredPumpPSIPacket createAtlasDesiredPumpPSIPacket(int desiredPumpPsi)
   {
      AtlasDesiredPumpPSIPacket message = new AtlasDesiredPumpPSIPacket();
      message.setDesiredPumpPsi(desiredPumpPsi);
      return message;
   }

   public static AtlasElectricMotorAutoEnableFlagPacket createAtlasElectricMotorAutoEnableFlagPacket(boolean shouldAutoEnable)
   {
      AtlasElectricMotorAutoEnableFlagPacket message = new AtlasElectricMotorAutoEnableFlagPacket();
      message.setShouldAutoEnable(shouldAutoEnable);
      return message;
   }

   public static EuclideanTrajectoryPointMessage createEuclideanTrajectoryPointMessage(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      EuclideanTrajectoryPointMessage message = new EuclideanTrajectoryPointMessage();
      message.setTime(time);
      message.getPosition().set(new Point3D(position));
      message.getLinearVelocity().set(new Vector3D(linearVelocity));
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket(WalkToGoalAction action)
   {
      WalkToGoalBehaviorPacket message = new WalkToGoalBehaviorPacket();
      message.setWalkToGoalAction(action.toByte());
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide)
   {
      WalkToGoalBehaviorPacket message = new WalkToGoalBehaviorPacket();
      message.setWalkToGoalAction(WalkToGoalAction.FIND_PATH.toByte());
      message.setXGoal(xGoal);
      message.setYGoal(yGoal);
      message.setThetaGoal(thetaGoal);
      message.setGoalRobotSide(goalSide.toByte());
      return message;
   }

   public static BDIBehaviorStatusPacket createBDIBehaviorStatusPacket(BDIRobotBehavior currentBehavior)
   {
      BDIBehaviorStatusPacket message = new BDIBehaviorStatusPacket();
      message.setCurrentBdiRobotBehavior(currentBehavior.toByte());
      return message;
   }

   public static SpineTrajectoryMessage createSpineTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of joints.
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      return createSpineTrajectoryMessage(trajectoryTime, desiredJointPositions, null, null);
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of joints.
    * @param weights               the qp weights for the joint accelerations. If any index is set to
    *                              NaN, that joint will use the controller default weight
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      return createSpineTrajectoryMessage(trajectoryTime, desiredJointPositions, null, weights);
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds  desired joint positions. The array length should be equal to the number of
    *                       joints.
    * @param weights        the qp weights for the joint accelerations. If any index is set to NaN,
    *                       that joint will use the controller default weight
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] desiredJointVelocities,
                                                                     double[] weights)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.getJointspaceTrajectory()
             .set(HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, desiredJointVelocities, weights));
      return message;
   }

   public static AutomaticManipulationAbortMessage createAutomaticManipulationAbortMessage(boolean enable)
   {
      AutomaticManipulationAbortMessage message = new AutomaticManipulationAbortMessage();
      message.setEnable(enable);
      return message;
   }

   public static StampedPosePacket createStampedPosePacket(String frameId, TimeStampedTransform3D transform, double confidenceFactor)
   {
      StampedPosePacket message = new StampedPosePacket();
      message.getFrameId().append(frameId);
      message.getPose().set(transform.getTransform3D());
      message.setTimestamp(transform.getTimeStamp());
      message.setConfidenceFactor(confidenceFactor);
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 long trajectoryReferenceFrameId)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroVector3D, zeroVector3D, trajectoryReferenceFrameId);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredLinearVelocity, Vector3DReadOnly desiredAngularVelocity,
                                                                 long trajectoryReferenceFrameId)
   {
      SE3TrajectoryMessage message = new SE3TrajectoryMessage();
      message.getTaskspaceTrajectoryPoints().add()
             .set(createSE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));
      message.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Pose3DReadOnly desiredPose, ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation(), trajectoryReferenceFrame);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Pose3DReadOnly desiredPose, SpatialVectorReadOnly desiredVelocity,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime,
                                        desiredPose.getPosition(),
                                        desiredPose.getOrientation(),
                                        desiredVelocity.getLinearPart(),
                                        desiredVelocity.getAngularPart(),
                                        trajectoryReferenceFrame);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroVector3D, zeroVector3D, trajectoryReferenceFrame);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredLinearVelocity, Vector3DReadOnly desiredAngularVelocity,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime,
                                        desiredPosition,
                                        desiredOrientation,
                                        desiredLinearVelocity,
                                        desiredAngularVelocity,
                                        trajectoryReferenceFrame.hashCode());
   }

   public static void configureForStreaming(WholeBodyTrajectoryMessage messageToModify, double streamIntegrationDuration)
   {
      configureForStreaming(messageToModify.getHeadTrajectoryMessage().getSo3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getChestTrajectoryMessage().getSo3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getPelvisTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getLeftArmTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getRightArmTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getLeftFootTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getRightFootTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getLeftHandTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getRightHandTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration);
      configureForStreaming(messageToModify.getSpineTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration);
   }

   public static void configureForStreaming(JointspaceTrajectoryMessage messageToModify, double streamIntegrationDuration)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration);
   }

   public static void configureForStreaming(EuclideanTrajectoryMessage messageToModify, double streamIntegrationDuration)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration);
   }

   public static void configureForStreaming(SO3TrajectoryMessage messageToModify, double streamIntegrationDuration)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration);
   }

   public static void configureForStreaming(SE3TrajectoryMessage messageToModify, double streamIntegrationDuration)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration);
   }

   public static void configureForStreaming(QueueableMessage messageToModify, double streamIntegrationDuration)
   {
      messageToModify.setExecutionMode(ExecutionMode.STREAM.toByte());
      messageToModify.setStreamIntegrationDuration(streamIntegrationDuration);
   }

   public static DetectedObjectPacket createDetectedObjectPacket(Pose3D pose, int id)
   {
      DetectedObjectPacket message = new DetectedObjectPacket();
      message.getPose().set(pose);
      message.setId(id);
      return message;
   }

   public static WalkingControllerFailureStatusMessage createWalkingControllerFailureStatusMessage(Vector2D fallingDirection)
   {
      WalkingControllerFailureStatusMessage message = new WalkingControllerFailureStatusMessage();
      message.getFallingDirection().set(fallingDirection);
      return message;
   }

   public static AtlasWristSensorCalibrationRequestPacket createAtlasWristSensorCalibrationRequestPacket(RobotSide robotSide)
   {
      AtlasWristSensorCalibrationRequestPacket message = new AtlasWristSensorCalibrationRequestPacket();
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static GoHomeMessage createGoHomeMessage(HumanoidBodyPart bodyPart, double trajectoryTime)
   {
      GoHomeMessage message = new GoHomeMessage();
      HumanoidMessageTools.checkRobotSide(bodyPart);
      message.setHumanoidBodyPart(bodyPart.toByte());
      message.setTrajectoryTime(trajectoryTime);
      return message;
   }

   public static GoHomeMessage createGoHomeMessage(HumanoidBodyPart bodyPart, RobotSide robotSide, double trajectoryTime)
   {
      GoHomeMessage message = new GoHomeMessage();
      if (robotSide == null)
         HumanoidMessageTools.checkRobotSide(bodyPart);
      message.setHumanoidBodyPart(bodyPart.toByte());
      message.setRobotSide(robotSide.toByte());
      message.setTrajectoryTime(trajectoryTime);
      return message;
   }

   public static SE3TrajectoryPointMessage createSE3TrajectoryPointMessage(double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                                                           Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      SE3TrajectoryPointMessage message = new SE3TrajectoryPointMessage();
      message.setTime(time);
      message.getPosition().set(position);
      message.getOrientation().set(orientation);
      message.getLinearVelocity().set(linearVelocity);
      message.getAngularVelocity().set(angularVelocity);
      return message;
   }

   public static WrenchTrajectoryPointMessage createWrenchTrajectoryPointMessage(double time, Vector3DReadOnly torque, Vector3DReadOnly force)
   {
      WrenchTrajectoryPointMessage message = new WrenchTrajectoryPointMessage();
      message.setTime(time);
      if (torque != null)
         message.getWrench().getTorque().set(torque);
      if (force != null)
         message.getWrench().getForce().set(force);
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Pose3DReadOnly pose)
   {
      return createFootstepDataMessage(robotSide, pose.getPosition(), pose.getOrientation());
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation,
                                                               List<? extends Point2DReadOnly> predictedContactPoints)
   {
      return createFootstepDataMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation,
                                                               TrajectoryType trajectoryType, double swingHeight)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation,
                                                               List<? extends Point2DReadOnly> predictedContactPoints, TrajectoryType trajectoryType,
                                                               double swingHeight)
   {
      FootstepDataMessage message = new FootstepDataMessage();
      message.setRobotSide(robotSide.toByte());
      message.getLocation().set(location);
      message.getOrientation().set(orientation);
      packPredictedContactPoints(predictedContactPoints, message);
      message.setTrajectoryType(trajectoryType.toByte());
      message.setSwingHeight(swingHeight);
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage(Footstep footstep)
   {
      FootstepDataMessage message = new FootstepDataMessage();

      message.setRobotSide(footstep.getRobotSide().toByte());

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      message.getLocation().set(location);
      message.getOrientation().set(orientation);
      packPredictedContactPoints(footstep.getPredictedContactPoints(), message);
      message.setTrajectoryType(footstep.getTrajectoryType().toByte());
      message.setSwingHeight(footstep.getSwingHeight());
      message.setSwingTrajectoryBlendDuration(footstep.getSwingTrajectoryBlendDuration());

      if (footstep.getCustomPositionWaypoints().size() != 0)
      {
         for (int i = 0; i < footstep.getCustomPositionWaypoints().size(); i++)
         {
            FramePoint3D framePoint = footstep.getCustomPositionWaypoints().get(i);
            framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
            message.getCustomPositionWaypoints().add().set(framePoint);
         }
      }

      return message;
   }

   public static KinematicsPlanningToolboxRigidBodyMessage createKinematicsPlanningToolboxRigidBodyMessage(RigidBodyBasics endEffector)
   {
      KinematicsPlanningToolboxRigidBodyMessage message = new KinematicsPlanningToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      return message;
   }

   public static KinematicsPlanningToolboxRigidBodyMessage createKinematicsPlanningToolboxRigidBodyMessage(RigidBodyBasics endEffector,
                                                                                                           TDoubleArrayList keyFrameTimes,
                                                                                                           List<Pose3DReadOnly> keyFramePoses)
   {
      KinematicsPlanningToolboxRigidBodyMessage message = new KinematicsPlanningToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());

      if (keyFrameTimes.size() != keyFramePoses.size())
         throw new RuntimeException("Inconsistent list lengths: keyFrameTimes.size() = " + keyFrameTimes.size() + ", keyFramePoses.size() = "
               + keyFramePoses.size());

      for (int i = 0; i < keyFrameTimes.size(); i++)
      {
         message.getKeyFrameTimes().add(keyFrameTimes.get(i));
         message.getKeyFramePoses().add().set(keyFramePoses.get(i));
      }
      KinematicsPlanningToolboxMessageFactory.setDefaultAllowableDisplacement(message, keyFrameTimes.size());

      return message;
   }

   public static KinematicsPlanningToolboxRigidBodyMessage createKinematicsPlanningToolboxRigidBodyMessage(RigidBodyBasics endEffector,
                                                                                                           ReferenceFrame controlFrame,
                                                                                                           TDoubleArrayList keyFrameTimes,
                                                                                                           List<Pose3DReadOnly> keyFramePoses)
   {
      KinematicsPlanningToolboxRigidBodyMessage message = new KinematicsPlanningToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());

      RigidBodyTransform transformToBodyFixedFrame = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(transformToBodyFixedFrame, endEffector.getBodyFixedFrame());
      message.getControlFramePositionInEndEffector().set(transformToBodyFixedFrame.getTranslationVector());
      message.getControlFrameOrientationInEndEffector().set(transformToBodyFixedFrame.getRotationMatrix());

      if (keyFrameTimes.size() != keyFramePoses.size())
         throw new RuntimeException("Inconsistent list lengths: keyFrameTimes.size() = " + keyFrameTimes.size() + ", keyFramePoses.size() = "
               + keyFramePoses.size());

      for (int i = 0; i < keyFrameTimes.size(); i++)
      {
         message.getKeyFrameTimes().add(keyFrameTimes.get(i));
         message.getKeyFramePoses().add().set(keyFramePoses.get(i));
      }
      KinematicsPlanningToolboxMessageFactory.setDefaultAllowableDisplacement(message, keyFrameTimes.size());

      return message;
   }

   public static KinematicsPlanningToolboxCenterOfMassMessage createKinematicsPlanningToolboxCenterOfMassMessage(TDoubleArrayList keyFrameTimes,
                                                                                                                 List<Point3DReadOnly> keyFramePoints)
   {
      KinematicsPlanningToolboxCenterOfMassMessage message = new KinematicsPlanningToolboxCenterOfMassMessage();
      if (keyFrameTimes.size() != keyFramePoints.size())
         throw new RuntimeException("Inconsistent list lengths: keyFrameTimes.size() = " + keyFrameTimes.size() + ", keyFramePoints.size() = "
               + keyFramePoints.size());
      for (int i = 0; i < keyFrameTimes.size(); i++)
      {
         message.getWayPointTimes().add(keyFrameTimes.get(i));
         message.getDesiredWayPointPositionsInWorld().add().set(keyFramePoints.get(i));
      }
      return message;
   }

   public static KinematicsPlanningToolboxOutputStatus createKinematicsPlanningToolboxOutputStatus()
   {
      KinematicsPlanningToolboxOutputStatus message = new KinematicsPlanningToolboxOutputStatus();
      return message;
   }

   public static PlanOffsetStatus createPlanOffsetStatus(Vector3DReadOnly offsetVector)
   {
      PlanOffsetStatus message = new PlanOffsetStatus();
      message.getOffsetVector().set(offsetVector);
      return message;
   }

   /**
    * set the class you want to clear
    *
    * @param clazz the class you want to clear
    */
   public static ClearDelayQueueMessage createClearDelayQueueMessage(Class<? extends Packet<?>> clazz)
   {
      ClearDelayQueueMessage message = new ClearDelayQueueMessage();
      message.setClassSimpleNameBasedHashCode(clazz.getSimpleName().hashCode());
      return message;
   }

   public static LocalizationStatusPacket createLocalizationStatusPacket(double overlap, String status)
   {
      LocalizationStatusPacket message = new LocalizationStatusPacket();
      message.setOverlap(overlap);
      message.setStatus(status);
      return message;
   }

   public static ValveLocationPacket createValveLocationPacket(RigidBodyTransform valveTransformToWorld, double valveRadius)
   {
      ValveLocationPacket message = new ValveLocationPacket();
      message.getValvePoseInWorld().set(valveTransformToWorld);
      message.setValveRadius(valveRadius);
      return message;
   }

   public static ValveLocationPacket createValveLocationPacket(Pose3D valvePoseInWorld, double valveRadius)
   {
      ValveLocationPacket message = new ValveLocationPacket();
      message.getValvePoseInWorld().set(valvePoseInWorld);
      message.setValveRadius(valveRadius);
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.getSe3Trajectory().set(trajectoryMessage);
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the
    * base for the control. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide          is used to define which foot is performing the trajectory.
    * @param trajectoryTime     how long it takes to reach the desired pose.
    * @param desiredPosition    desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.getSe3Trajectory()
             .set(HumanoidMessageTools.createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, ReferenceFrame.getWorldFrame()));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Pose3DReadOnly desiredPose)
   {
      return createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation());
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Pose3DReadOnly desiredPose,
                                                                   SpatialVectorReadOnly desiredVelocity, ReferenceFrame trajectoryReferenceFrame)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPose, desiredVelocity, trajectoryReferenceFrame));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static PrepareForLocomotionMessage createPrepareForLocomotionMessage(boolean prepareManipulation, boolean preparePelvis)
   {
      PrepareForLocomotionMessage message = new PrepareForLocomotionMessage();
      message.setPrepareManipulation(prepareManipulation);
      message.setPreparePelvis(preparePelvis);
      return message;
   }

   public static WalkOverTerrainGoalPacket createWalkOverTerrainGoalPacket(Point3D position, Quaternion orientation)
   {
      WalkOverTerrainGoalPacket message = new WalkOverTerrainGoalPacket();
      message.getPosition().set(position);
      message.getOrientation().set(orientation);
      return message;
   }

   public static void checkRobotSide(HumanoidBodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
   }

   public static IntrinsicParametersMessage toIntrinsicParametersMessage(IntrinsicParameters intrinsicParameters)
   {
      IntrinsicParametersMessage intrinsicParametersMessage = new IntrinsicParametersMessage();
      intrinsicParametersMessage.setWidth(intrinsicParameters.width);
      intrinsicParametersMessage.setHeight(intrinsicParameters.height);
      intrinsicParametersMessage.setFx(intrinsicParameters.fx);
      intrinsicParametersMessage.setFy(intrinsicParameters.fy);
      intrinsicParametersMessage.setSkew(intrinsicParameters.skew);
      intrinsicParametersMessage.setCx(intrinsicParameters.cx);
      intrinsicParametersMessage.setCy(intrinsicParameters.cy);
      if (intrinsicParameters.radial != null)
         intrinsicParametersMessage.getRadial().add(intrinsicParameters.radial);
      intrinsicParametersMessage.setT1(intrinsicParameters.t1);
      intrinsicParametersMessage.setT2(intrinsicParameters.t2);
      return intrinsicParametersMessage;
   }

   public static IntrinsicParameters toIntrinsicParameters(IntrinsicParametersMessage message)
   {
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters();
      intrinsicParameters.width = message.getWidth();
      intrinsicParameters.height = message.getHeight();
      intrinsicParameters.fx = message.getFx();
      intrinsicParameters.fy = message.getFy();
      intrinsicParameters.skew = message.getSkew();
      intrinsicParameters.cx = message.getCx();
      intrinsicParameters.cy = message.getCy();
      if (!message.getRadial().isEmpty())
         intrinsicParameters.radial = message.getRadial().toArray();
      intrinsicParameters.t1 = message.getT1();
      intrinsicParameters.t2 = message.getT2();
      return intrinsicParameters;
   }

   public static void packPredictedContactPoints(Point2DReadOnly[] contactPoints, FootstepDataMessage message)
   {
      if (contactPoints == null)
         return;
      MessageTools.copyData(Arrays.stream(contactPoints).map(Point3D::new).collect(Collectors.toList()), message.getPredictedContactPoints2d());
   }

   public static void packPredictedContactPoints(List<? extends Point2DReadOnly> contactPoints, FootstepDataMessage message)
   {
      if (contactPoints == null)
         return;

      message.getPredictedContactPoints2d().clear();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         message.getPredictedContactPoints2d().add().set(contactPoints.get(i), 0.0);
      }
   }

   public static List<Point2D> unpackPredictedContactPoints(FootstepDataMessage message)
   {
      return message.getPredictedContactPoints2d().stream().map(Point2D::new).collect(Collectors.toList());
   }

   public static void setGroundQuadTreeSupport(Point3DReadOnly[] pointCloud, PointCloudWorldPacket pointCloudWorldPacket)
   {
      pointCloudWorldPacket.getGroundQuadTreeSupport().reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         pointCloudWorldPacket.getGroundQuadTreeSupport().add((float) point.getX());
         pointCloudWorldPacket.getGroundQuadTreeSupport().add((float) point.getY());
         pointCloudWorldPacket.getGroundQuadTreeSupport().add((float) point.getZ());
      }
   }

   public static void setDecayingWorldScan(Point3DReadOnly[] pointCloud, PointCloudWorldPacket pointCloudWorldPacket)
   {
      pointCloudWorldPacket.getDecayingWorldScan().reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         pointCloudWorldPacket.getDecayingWorldScan().add((float) point.getX());
         pointCloudWorldPacket.getDecayingWorldScan().add((float) point.getY());
         pointCloudWorldPacket.getDecayingWorldScan().add((float) point.getZ());
      }
   }

   public static Point3D32[] getDecayingWorldScan(PointCloudWorldPacket pointCloudWorldPacket)
   {
      int numberOfPoints = pointCloudWorldPacket.getDecayingWorldScan().size() / 3;

      Point3D32[] points = new Point3D32[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(pointCloudWorldPacket.getDecayingWorldScan().get(3 * i));
         point.setY(pointCloudWorldPacket.getDecayingWorldScan().get(3 * i + 1));
         point.setZ(pointCloudWorldPacket.getDecayingWorldScan().get(3 * i + 2));
         points[i] = point;
      }

      return points;
   }

   public static HeightQuadTreeToolboxRequestMessage clearRequest(PacketDestination destination)
   {
      HeightQuadTreeToolboxRequestMessage clearMessage = new HeightQuadTreeToolboxRequestMessage();
      clearMessage.setDestination(destination.ordinal());
      clearMessage.setRequestClearQuadTree(true);
      clearMessage.setRequestQuadTreeUpdate(false);
      return clearMessage;
   }

   public static HeightQuadTreeToolboxRequestMessage requestQuadTreeUpdate(PacketDestination destination)
   {
      HeightQuadTreeToolboxRequestMessage requestMessage = new HeightQuadTreeToolboxRequestMessage();
      requestMessage.setDestination(destination.ordinal());
      requestMessage.setRequestClearQuadTree(false);
      requestMessage.setRequestQuadTreeUpdate(true);
      return requestMessage;
   }

   public static void packLocalizationPointMap(Point3DReadOnly[] pointCloud, LocalizationPointMapPacket localizationPointMapPacket)
   {
      localizationPointMapPacket.getLocalizationPointMap().reset();

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly point = pointCloud[i];
         localizationPointMapPacket.getLocalizationPointMap().add((float) point.getX());
         localizationPointMapPacket.getLocalizationPointMap().add((float) point.getY());
         localizationPointMapPacket.getLocalizationPointMap().add((float) point.getZ());
      }
   }

   public static Point3D32[] unpackLocalizationPointMap(LocalizationPointMapPacket localizationPointMapPacket)
   {
      int numberOfPoints = localizationPointMapPacket.getLocalizationPointMap().size() / 3;

      Point3D32[] points = new Point3D32[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(localizationPointMapPacket.getLocalizationPointMap().get(3 * i));
         point.setY(localizationPointMapPacket.getLocalizationPointMap().get(3 * i + 1));
         point.setZ(localizationPointMapPacket.getLocalizationPointMap().get(3 * i + 2));
         points[i] = point;
      }

      return points;
   }

   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, ReferenceFrame referenceFrame)
   {
      long expectedId = HumanoidMessageTools.getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != referenceFrame.hashCode() && expectedId != referenceFrame.getAdditionalNameBasedHashCode())
      {
         String msg = "Argument's hashcode " + referenceFrame + " " + referenceFrame.hashCode() + " does not match " + expectedId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, long otherReferenceFrameId)
   {
      long expectedId = HumanoidMessageTools.getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != otherReferenceFrameId)
      {
         String msg = "Argument's hashcode " + otherReferenceFrameId + " does not match " + expectedId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static long getDataFrameIDConsideringDefault(FrameInformation frameInformation)
   {
      long dataId = frameInformation.getDataReferenceFrameId();
      if (dataId == NameBasedHashCodeTools.DEFAULT_HASHCODE)
      {
         dataId = frameInformation.getTrajectoryReferenceFrameId();
      }
      return dataId;
   }

   public static double unpackJointAngle(HandJointAnglePacket handJointAnglePacket, HandJointName jointName)
   {
      int index = jointName.getIndex(RobotSide.fromByte(handJointAnglePacket.getRobotSide()));
      if (index == -1)
      {
         return 0;
      }

      return handJointAnglePacket.getJointAngles().get(index);
   }

   public static void packFootSupportPolygon(RobotSide robotSide, ConvexPolygon2DReadOnly footPolygon, CapturabilityBasedStatus capturabilityBasedStatus)
   {
      int numberOfVertices = footPolygon.getNumberOfVertices();

      if (numberOfVertices > CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES)
      {
         numberOfVertices = CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES;
      }

      if (robotSide == RobotSide.LEFT)
      {
         capturabilityBasedStatus.getLeftFootSupportPolygon2d().clear();
      }
      else
      {
         capturabilityBasedStatus.getRightFootSupportPolygon2d().clear();
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         if (robotSide == RobotSide.LEFT)
         {
            capturabilityBasedStatus.getLeftFootSupportPolygon2d().add().set(footPolygon.getVertex(i), 0.0);
         }
         else
         {
            capturabilityBasedStatus.getRightFootSupportPolygon2d().add().set(footPolygon.getVertex(i), 0.0);
         }
      }
   }

   public static FrameConvexPolygon2D unpackFootSupportPolygon(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT && capturabilityBasedStatus.getLeftFootSupportPolygon2d().size() > 0)
         return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame(),
                                         Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getLeftFootSupportPolygon2d()));
      else if (capturabilityBasedStatus.getRightFootSupportPolygon2d() != null)
         return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame(),
                                         Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getRightFootSupportPolygon2d()));
      else
         return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame());
   }

   public static boolean unpackIsInDoubleSupport(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      return capturabilityBasedStatus.getLeftFootSupportPolygon2d().size() != 0 & capturabilityBasedStatus.getRightFootSupportPolygon2d().size() != 0;
   }

   public static boolean unpackIsSupportFoot(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotside)
   {
      if (robotside == RobotSide.LEFT)
         return capturabilityBasedStatus.getLeftFootSupportPolygon2d().size() != 0;
      else
         return capturabilityBasedStatus.getRightFootSupportPolygon2d().size() != 0;
   }

   public static void packManifold(byte[] configurationSpaces, double[] lowerLimits, double[] upperLimits, ReachingManifoldMessage reachingManifoldMessage)
   {
      if (configurationSpaces.length != lowerLimits.length || configurationSpaces.length != upperLimits.length || lowerLimits.length != upperLimits.length)
         throw new RuntimeException("Inconsistent array lengths: configurationSpaces = " + configurationSpaces.length);

      reachingManifoldMessage.getManifoldConfigurationSpaceNames().reset();
      reachingManifoldMessage.getManifoldLowerLimits().reset();
      reachingManifoldMessage.getManifoldUpperLimits().reset();
      reachingManifoldMessage.getManifoldConfigurationSpaceNames().add(configurationSpaces);
      reachingManifoldMessage.getManifoldLowerLimits().add(lowerLimits);
      reachingManifoldMessage.getManifoldUpperLimits().add(upperLimits);
   }

   public static Pose3D unpackPose(WaypointBasedTrajectoryMessage waypointBasedTrajectoryMessage, double time)
   {
      if (time <= 0.0)
         return waypointBasedTrajectoryMessage.getWaypoints().get(0);

      else if (time >= waypointBasedTrajectoryMessage.getWaypointTimes().get(waypointBasedTrajectoryMessage.getWaypointTimes().size() - 1))
         return waypointBasedTrajectoryMessage.getWaypoints().getLast();

      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = waypointBasedTrajectoryMessage.getWaypointTimes().size();

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - waypointBasedTrajectoryMessage.getWaypointTimes().get(i);
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         Pose3D poseOne = waypointBasedTrajectoryMessage.getWaypoints().get(indexOfFrame - 1);
         Pose3D poseTwo = waypointBasedTrajectoryMessage.getWaypoints().get(indexOfFrame);

         double timeOne = waypointBasedTrajectoryMessage.getWaypointTimes().get(indexOfFrame - 1);
         double timeTwo = waypointBasedTrajectoryMessage.getWaypointTimes().get(indexOfFrame);

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         Pose3D ret = new Pose3D();
         ret.interpolate(poseOne, poseTwo, alpha);

         return ret;
      }
   }

   public static double unpackTrajectoryTime(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      double trajectoryTime = 0.0;
      for (int i = 0; i < jointspaceTrajectoryMessage.getJointTrajectoryMessages().size(); i++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(i);
         if (oneDoFJointTrajectoryMessage != null && !oneDoFJointTrajectoryMessage.getTrajectoryPoints().isEmpty())
         {
            trajectoryTime = Math.max(trajectoryTime, oneDoFJointTrajectoryMessage.getTrajectoryPoints().getLast().getTime());
         }
      }
      return trajectoryTime;
   }
}
