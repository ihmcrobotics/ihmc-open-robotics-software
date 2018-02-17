package us.ihmc.humanoidRobotics.communication.packets;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlModeMessage;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorCommandPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeResponsePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DoorLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkOverTerrainGoalPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalAction;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WallPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasDesiredPumpPSIPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorAutoEnableFlagPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorEnablePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasWristSensorCalibrationRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPowerCyclePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ReachingManifoldMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.momentum.CenterOfMassTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.momentum.MomentumTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.FisheyePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseParameterPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorModePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanOffsetStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PrepareForLocomotionMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.ClearDelayQueueMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;

public class HumanoidMessageTools
{
   private HumanoidMessageTools()
   {
   }

   public static BlackFlyParameterPacket createBlackFlyParameterPacket(boolean fromUI, double gain, double exposure, double frameRate, double shutter,
                                                                       boolean autoExposure, boolean autoGain, boolean autoShutter, RobotSide side)
   {
      BlackFlyParameterPacket message = new BlackFlyParameterPacket();
      message.fromUI = fromUI;
      message.exposure = exposure;
      message.shutter = shutter;
      message.gain = gain;
      message.frameRate = frameRate;
      message.autoExposure = autoExposure;
      message.autoGain = autoGain;
      message.autoShutter = autoShutter;
      message.robotSide = side.toByte();
      return message;
   }

   public static DesiredAccelerationsMessage createDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      DesiredAccelerationsMessage message = new DesiredAccelerationsMessage();
      message.desiredJointAccelerations = desiredJointAccelerations;
      return message;
   }

   public static NeckDesiredAccelerationsMessage createNeckDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      NeckDesiredAccelerationsMessage message = new NeckDesiredAccelerationsMessage();
      message.desiredAccelerations = HumanoidMessageTools.createDesiredAccelerationsMessage(desiredJointAccelerations);
      return message;
   }

   public static ChestHybridJointspaceTaskspaceTrajectoryMessage createChestHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                       JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ChestHybridJointspaceTaskspaceTrajectoryMessage message = new ChestHybridJointspaceTaskspaceTrajectoryMessage();
      message.taskspaceTrajectoryMessage = new SO3TrajectoryMessage(taskspaceTrajectoryMessage);
      message.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   public static HeadHybridJointspaceTaskspaceTrajectoryMessage createHeadHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                     JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      HeadHybridJointspaceTaskspaceTrajectoryMessage message = new HeadHybridJointspaceTaskspaceTrajectoryMessage();
      message.taskspaceTrajectoryMessage = new SO3TrajectoryMessage(taskspaceTrajectoryMessage);
      message.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   public static HandHybridJointspaceTaskspaceTrajectoryMessage createHandHybridJointspaceTaskspaceTrajectoryMessage(RobotSide robotSide,
                                                                                                                     SE3TrajectoryMessage taskspaceTrajectoryMessage,
                                                                                                                     JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      HandHybridJointspaceTaskspaceTrajectoryMessage message = new HandHybridJointspaceTaskspaceTrajectoryMessage();
      message.robotSide = robotSide.toByte();
      message.taskspaceTrajectoryMessage = new SE3TrajectoryMessage(taskspaceTrajectoryMessage);
      message.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      message.robotSide = robotSide.toByte();
      message.setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of arm joints.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of arm joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that
    *           joint will use the controller default weight
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, weights);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(jointTrajectory1DListMessages);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(numberOfJoints);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory points. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory
    * point afterwards. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints, int numberOfTrajectoryPoints)
   {
      ArmTrajectoryMessage message = new ArmTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(numberOfJoints, numberOfTrajectoryPoints);
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.se3Trajectory = new SE3TrajectoryMessage(trajectoryMessage);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as
    * the base for the control. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameId)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.se3Trajectory = createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrameId);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as
    * the base for the control. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.se3Trajectory = createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. By default this
    * constructor sets the trajectory frame to {@link CommonReferenceFrameIds#CHEST_FRAME} and the
    * data frame to World This constructor only allocates memory for the trajectory points, you need
    * to call
    * {@link #setTrajectoryPoint(int, double, Point3D, QuaternionReadOnly, Vector3D, Vector3D)} for
    * each trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.se3Trajectory = createSE3TrajectoryMessage(numberOfTrajectoryPoints);
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static BehaviorStatusPacket createBehaviorStatusPacket(CurrentBehaviorStatus requestedControl)
   {
      BehaviorStatusPacket message = new BehaviorStatusPacket();
      message.currentBehaviorStatus = requestedControl.toByte();
      return message;
   }

   public static LegCompliancePacket createLegCompliancePacket(float[] maxVelocityDeltas, RobotSide robotSide)
   {
      LegCompliancePacket message = new LegCompliancePacket();
      message.maxVelocityDeltas = maxVelocityDeltas;
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static SnapFootstepPacket createSnapFootstepPacket(ArrayList<FootstepDataMessage> data, int[] footstepOrder, byte[] flag)
   {
      SnapFootstepPacket message = new SnapFootstepPacket();
      message.footstepData = data;
      message.footstepOrder = footstepOrder;
      message.flag = flag;
      return message;
   }

   public static BehaviorControlModePacket createBehaviorControlModePacket(BehaviorControlModeEnum requestedControl)
   {
      BehaviorControlModePacket message = new BehaviorControlModePacket();
      message.behaviorControlModeEnumRequest = requestedControl.toByte();
      return message;
   }

   /**
    * Create a message to request one end-effector to switch to load bearing. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide refers to the side of the end-effector if necessary.
    */
   public static FootLoadBearingMessage createFootLoadBearingMessage(RobotSide robotSide, LoadBearingRequest request)
   {
      FootLoadBearingMessage message = new FootLoadBearingMessage();
      message.robotSide = robotSide.toByte();
      message.loadBearingRequest = request.toByte();
      return message;
   }

   // joint values should be in the range [0,1]
   public static ManualHandControlPacket createManualHandControlPacket(RobotSide robotSide, double index, double middle, double thumb, double spread,
                                                                       int controlType)
   {
      ManualHandControlPacket message = new ManualHandControlPacket();
      message.robotSide = robotSide.toByte();
      message.index = index;
      message.middle = middle;
      message.thumb = thumb;
      message.spread = spread;
      message.controlType = controlType;
      return message;
   }

   public static MultisenseParameterPacket createMultisenseParameterPacket(boolean initialize, double gain, double motorSpeed, double dutyCycle,
                                                                           boolean ledEnable, boolean flashEnable, boolean autoExposure,
                                                                           boolean autoWhiteBalance)
   {
      MultisenseParameterPacket message = new MultisenseParameterPacket();
      message.initialize = initialize;
      message.gain = gain;
      message.flashEnable = flashEnable;
      message.motorSpeed = motorSpeed;
      message.ledEnable = ledEnable;
      message.dutyCycle = dutyCycle;
      message.autoExposure = autoExposure;
      message.autoWhiteBalance = autoWhiteBalance;
      return message;
   }

   public static DoorLocationPacket createDoorLocationPacket(RigidBodyTransform doorTransformToWorld)
   {
      DoorLocationPacket message = new DoorLocationPacket();
      message.doorTransformToWorld = doorTransformToWorld;
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket(Point3D position, Quaternion orientation)
   {
      VehiclePosePacket message = new VehiclePosePacket();
      message.position = position;
      message.orientation = orientation;
      return message;
   }

   public static VehiclePosePacket createVehiclePosePacket(RigidBodyTransform transformFromVehicleToWorld)
   {
      VehiclePosePacket message = new VehiclePosePacket();
      message.orientation = new Quaternion(transformFromVehicleToWorld.getRotationMatrix());
      message.position = new Point3D(transformFromVehicleToWorld.getTranslationVector());
      return message;
   }

   public static HighLevelStateChangeStatusMessage createHighLevelStateChangeStatusMessage(HighLevelControllerName initialState,
                                                                                           HighLevelControllerName endState)
   {
      HighLevelStateChangeStatusMessage message = new HighLevelStateChangeStatusMessage();
      message.setStateChange(initialState == null ? -1 : initialState.toByte(), endState == null ? -1 : endState.toByte());
      return message;
   }

   /**
    * To set disable exploration on this rigid body.
    */
   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      message.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();

      ConfigurationSpaceName[] configurations = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.YAW,
            ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL};
      double[] regionAmplitude = new double[] {0, 0, 0, 0, 0, 0};
      message.setExplorationConfigurationSpaces(ConfigurationSpaceName.toBytes(configurations), regionAmplitude);

      return message;
   }

   /**
    * To set enable exploration on this rigid body with following order of ConfigurationSpaceName.
    */
   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore)
   {
      return createRigidBodyExplorationConfigurationMessage(rigidBody, degreesOfFreedomToExplore,
                                                            WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationAmplitudeArray(degreesOfFreedomToExplore));
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                                                                         double[] explorationRangeAmplitudes)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      if (degreesOfFreedomToExplore.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      message.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      message.setExplorationConfigurationSpaces(ConfigurationSpaceName.toBytes(degreesOfFreedomToExplore), explorationRangeAmplitudes);

      return message;
   }

   public static RigidBodyExplorationConfigurationMessage createRigidBodyExplorationConfigurationMessage(RigidBody rigidBody,
                                                                                                         ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                                                                         double[] explorationRangeUpperLimits,
                                                                                                         double[] explorationRangeLowerLimits)
   {
      RigidBodyExplorationConfigurationMessage message = new RigidBodyExplorationConfigurationMessage();
      if (degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      message.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      message.setExplorationConfigurationSpaces(ConfigurationSpaceName.toBytes(degreesOfFreedomToExplore), explorationRangeUpperLimits, explorationRangeLowerLimits);

      return message;
   }

   public static FootstepPathPlanPacket createFootstepPathPlanPacket(boolean goalsValid, FootstepDataMessage start,
                                                                     ArrayList<FootstepDataMessage> originalGoals,
                                                                     ArrayList<FootstepDataMessage> ADStarPathPlan, ArrayList<Boolean> footstepUnknown,
                                                                     double subOptimality, double cost)
   {
      FootstepPathPlanPacket message = new FootstepPathPlanPacket();
      message.goalsValid = goalsValid;
      message.start = start;
      message.originalGoals = originalGoals;
      message.pathPlan = ADStarPathPlan;
      message.footstepUnknown = footstepUnknown;
      message.subOptimality = subOptimality;
      message.pathCost = cost;
      return message;
   }

   public static MomentumTrajectoryMessage createMomentumTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      MomentumTrajectoryMessage message = new MomentumTrajectoryMessage();
      message.angularMomentumTrajectory = createEuclideanTrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   public static ObjectWeightPacket createObjectWeightPacket(RobotSide robotSide, double weight)
   {
      ObjectWeightPacket message = new ObjectWeightPacket();
      message.robotSide = robotSide.toByte();
      message.weight = weight;
      return message;
   }

   public static HandPowerCyclePacket createHandPowerCyclePacket(RobotSide robotSide)
   {
      HandPowerCyclePacket message = new HandPowerCyclePacket();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints)
   {
      return createWaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints);
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints,
                                                                                     SelectionMatrix6D selectionMatrix)
   {
      WaypointBasedTrajectoryMessage message = new WaypointBasedTrajectoryMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setWaypoints(waypointTimes, waypoints);
      if (selectionMatrix != null)
      {
         message.angularSelectionMatrix.set(selectionMatrix.getAngularPart());
         message.linearSelectionMatrix.set(selectionMatrix.getLinearPart());
      }
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired pelvis position expressed in world frame.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                       QuaternionReadOnly desiredOrientation)
   {
      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.se3Trajectory = createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, ReferenceFrame.getWorldFrame());
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. This constructor only allocates memory for
    * the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Point3DReadOnly, QuaternionReadOnly, Vector3DReadOnly, Vector3DReadOnly)}
    * for each trajectory point afterwards.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.se3Trajectory = createSE3TrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   public static PelvisPoseErrorPacket createPelvisPoseErrorPacket(float residualError, float totalError, boolean hasMapBeenReset)
   {
      PelvisPoseErrorPacket message = new PelvisPoseErrorPacket();
      message.residualError = residualError;
      message.totalError = totalError;
      message.hasMapBeenReset = hasMapBeenReset;
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                                   ArrayList<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      AdjustFootstepMessage message = new AdjustFootstepMessage();
      message.robotSide = robotSide.toByte();
      message.location = location;
      message.orientation = orientation;
      if (predictedContactPoints != null && predictedContactPoints.size() == 0)
         message.predictedContactPoints = null;
      else
         message.predictedContactPoints = predictedContactPoints;
      return message;
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType,
                                                                   double swingHeight)
   {
      return createAdjustFootstepMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static AdjustFootstepMessage createAdjustFootstepMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                                   ArrayList<Point2D> predictedContactPoints)
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
      message.robotSide = footstep.getRobotSide().toByte();

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      message.location = new Point3D(location);
      message.orientation = new Quaternion(orientation);

      List<Point2D> footstepContactPoints = footstep.getPredictedContactPoints();
      if (footstepContactPoints != null)
      {
         if (message.predictedContactPoints == null)
         {
            message.predictedContactPoints = new ArrayList<>();
         }
         else
         {
            message.predictedContactPoints.clear();
         }
         for (Point2D contactPoint : footstepContactPoints)
         {
            message.predictedContactPoints.add(new Point2D(contactPoint));
         }
      }
      else
      {
         message.predictedContactPoints = null;
      }
      return message;
   }

   public static NeckTrajectoryMessage createNeckTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of joints.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that
    *           joint will use the controller default weight
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, weights);
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
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(jointTrajectory1DListMessages);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(int numberOfJoints)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(numberOfJoints);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory points. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory
    * point afterwards. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      NeckTrajectoryMessage message = new NeckTrajectoryMessage();
      message.jointspaceTrajectory = createJointspaceTrajectoryMessage(numberOfJoints, numberOfTrajectoryPoints);
      return message;
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose)
   {
      FootstepPlanningRequestPacket message = new FootstepPlanningRequestPacket();
      message.set(initialStanceFootPose, initialStanceSide.toByte(), goalPose, FootstepPlannerType.PLANAR_REGION_BIPEDAL.toByte());
      return message;
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose, FootstepPlannerType requestedPlannerType)
   {
      FootstepPlanningRequestPacket message = new FootstepPlanningRequestPacket();
      message.set(initialStanceFootPose, initialStanceSide.toByte(), goalPose, requestedPlannerType.toByte());
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation towards the given endpoint. Set the id
    * of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame());
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. This constructor only allocates memory for
    * the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point
    * afterwards.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                                             ReferenceFrame trajectoryFrame)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      return message;
   }

   public static WholeBodyTrajectoryToolboxMessage createWholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxConfigurationMessage configuration,
                                                                                           List<WaypointBasedTrajectoryMessage> endEffectorTrajectories,
                                                                                           List<ReachingManifoldMessage> reachingManifolds,
                                                                                           List<RigidBodyExplorationConfigurationMessage> explorationConfigurations)
   {
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage();
      message.configuration = configuration;
      message.endEffectorTrajectories = endEffectorTrajectories;
      message.reachingManifolds = reachingManifolds;
      message.explorationConfigurations = explorationConfigurations;
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket(BDIRobotBehavior atlasRobotBehavior)
   {
      BDIBehaviorCommandPacket message = new BDIBehaviorCommandPacket();
      message.atlasBDIRobotBehavior = atlasRobotBehavior.toByte();
      return message;
   }

   public static AtlasElectricMotorEnablePacket createAtlasElectricMotorEnablePacket(AtlasElectricMotorPacketEnum motorEnableEnum, boolean enable)
   {
      AtlasElectricMotorEnablePacket message = new AtlasElectricMotorEnablePacket();
      message.atlasElectricMotorPacketEnumEnable = motorEnableEnum.toByte();
      message.enable = enable;
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(SO3TrajectoryMessage so3Trajectory)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.so3Trajectory = new SO3TrajectoryMessage(so3Trajectory);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed in World.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     long trajectoryReferenceFrameID)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryReferenceFrameID);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed the supplied frame.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime, QuaternionReadOnly quaternion, ReferenceFrame dataFrame,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage(trajectoryTime, quaternion, trajectoryFrame);
      message.so3Trajectory.getFrameInformation().setDataReferenceFrame(dataFrame);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. By default this
    * constructor sets the trajectory frame to pelvis z up and the data frame to world This
    * constructor only allocates memory for the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point
    * afterwards. Sets the frame to control in to world
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame dataFrame,
                                                                   ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      message.so3Trajectory.getFrameInformation().setDataReferenceFrame(dataFrame);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired
    * orientation. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation,
                                                                   long trajectoryReferenceFrameId)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryReferenceFrameId);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. By default this
    * constructor sets the trajectory frame to chest Center of mass frame and the data frame to
    * world This constructor only allocates memory for the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point
    * afterwards. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.so3Trajectory = createSO3TrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   /**
    * set a single point
    * 
    * @param trajectoryTime the duration of the trajectory
    * @param desiredPosition the desired end position
    * @param trajectoryReferenceFrameId the frame id the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             long trajectoryReferenceFrameId)
   {
      EuclideanTrajectoryMessage message = new EuclideanTrajectoryMessage();
      Vector3D zeroLinearVelocity = new Vector3D();
      message.taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[] {
            createEuclideanTrajectoryPointMessage(trajectoryTime, desiredPosition, zeroLinearVelocity)};
      message.frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   /**
    * set a single point
    * 
    * @param trajectoryTime the duration of the trajectory
    * @param desiredPosition the desired end position
    * @param trajectoryReferenceFrame the frame the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                             ReferenceFrame trajectoryReferenceFrame)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, trajectoryReferenceFrame.getNameBasedHashCode());
   }

   /**
    * creates a new empty message with a trajectory point list the size of numberOfTrajectoryPoints
    * 
    * @param numberOfTrajectoryPoints number of trajectory points in this message
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      EuclideanTrajectoryMessage message = new EuclideanTrajectoryMessage();
      message.taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[numberOfTrajectoryPoints];
      return message;
   }

   public static MessageOfMessages createMessageOfMessages(List<Packet<?>> messages)
   {
      MessageOfMessages message = new MessageOfMessages();
      message.packets.clear();
      message.packets.addAll(messages);
      return message;
   }

   public static MessageOfMessages createMessageOfMessages(Packet<?>... messages)
   {
      MessageOfMessages message = new MessageOfMessages();
      message.packets.clear();
      for (Packet<?> packet : messages)
      {
         message.packets.add(packet);
      }
      return message;
   }

   public static LocalizationPacket createLocalizationPacket(boolean reset, boolean toggle)
   {
      LocalizationPacket message = new LocalizationPacket();
      message.reset = reset;
      message.toggle = toggle;
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in data frame
    * @param trajectoryReferenceFrame the frame in which the height will be executed
    * @param dataReferenceFrame the frame the desiredHeight is expressed in, the height will be
    *           changed to the trajectory frame on the controller side
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight,
                                                                                   ReferenceFrame trajectoryReferenceFrame, ReferenceFrame dataReferenceFrame)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.euclideanTrajectory = HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                                                          trajectoryReferenceFrame.getNameBasedHashCode());
      message.euclideanTrajectory.frameInformation.setDataReferenceFrame(dataReferenceFrame);
      message.euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      message.euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. The trajectory and data frame are
    * set to world frame Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.euclideanTrajectory = HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                                                          ReferenceFrame.getWorldFrame());
      message.euclideanTrajectory.frameInformation.setDataReferenceFrame(ReferenceFrame.getWorldFrame());
      message.euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      message.euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint}
    * for each trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.euclideanTrajectory = HumanoidMessageTools.createEuclideanTrajectoryMessage(numberOfTrajectoryPoints);
      message.euclideanTrajectory.selectionMatrix = new SelectionMatrix3DMessage();
      message.euclideanTrajectory.selectionMatrix.setAxisSelection(false, false, true);
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = null;
      message.desiredFootOrientationInWorld = null;
      message.actualFootPositionInWorld = null;
      message.actualFootOrientationInWorld = null;
      message.robotSide = -1;
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D actualFootPositionInWorld,
                                                     Quaternion actualFootOrientationInWorld)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = null;
      message.desiredFootOrientationInWorld = null;
      message.actualFootPositionInWorld = actualFootPositionInWorld;
      message.actualFootOrientationInWorld = actualFootOrientationInWorld;

      message.robotSide = -1;
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D actualFootPositionInWorld,
                                                     Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = null;
      message.desiredFootOrientationInWorld = null;
      message.actualFootPositionInWorld = actualFootPositionInWorld;
      message.actualFootOrientationInWorld = actualFootOrientationInWorld;
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status, int footstepIndex, Point3D desiredFootPositionInWorld,
                                                     Quaternion desiredFootOrientationInWorld, Point3D actualFootPositionInWorld,
                                                     Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      FootstepStatusMessage message = new FootstepStatusMessage();
      message.footstepStatus = status.toByte();
      message.footstepIndex = footstepIndex;
      message.desiredFootPositionInWorld = desiredFootPositionInWorld;
      message.desiredFootOrientationInWorld = desiredFootOrientationInWorld;
      message.actualFootPositionInWorld = actualFootPositionInWorld;
      message.actualFootOrientationInWorld = actualFootOrientationInWorld;
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      return createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame.getNameBasedHashCode());
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameId)
   {
      SO3TrajectoryMessage message = new SO3TrajectoryMessage();
      Vector3D zeroAngularVelocity = new Vector3D();
      message.taskspaceTrajectoryPoints = new SO3TrajectoryPointMessage[] {
            createSO3TrajectoryPointMessage(trajectoryTime, desiredOrientation, zeroAngularVelocity)};
      message.frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(int numberOfTrajectoryPoints)
   {
      SO3TrajectoryMessage message = new SO3TrajectoryMessage();
      message.taskspaceTrajectoryPoints = new SO3TrajectoryPointMessage[numberOfTrajectoryPoints];
      return message;
   }

   public static HighLevelStateMessage createHighLevelStateMessage(HighLevelControllerName highLevelControllerName)
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.highLevelControllerName = highLevelControllerName.toByte();
      return message;
   }

   public static WallPosePacket createWallPosePacket(WallPosePacket other)
   {
      WallPosePacket message = new WallPosePacket();
      message.setCuttingRadius(other.getCuttingRadius());
      message.setCenterPosition(other.getCenterPosition());
      message.setCenterOrientation(other.getCenterOrientation());
      return message;
   }

   public static WallPosePacket createWallPosePacket(double cuttingRadius, Tuple3DReadOnly centerPosition, QuaternionReadOnly centerOrientation)
   {
      WallPosePacket message = new WallPosePacket();
      message.setCuttingRadius(cuttingRadius);
      message.setCenterPosition(centerPosition);
      message.setCenterOrientation(centerOrientation);
      return message;
   }

   public static WallPosePacket createWallPosePacket(double cuttingRadius, Tuple3DReadOnly centerPosition, RotationMatrixReadOnly rotationMatrix)
   {
      WallPosePacket message = new WallPosePacket();
      message.setCuttingRadius(cuttingRadius);
      message.setCenterPosition(centerPosition);
      message.setCenterOrientation(new Quaternion(rotationMatrix));
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket(FootstepPlanRequestType requestType, FootstepDataMessage startFootstep, double thetaStart,
                                                                           ArrayList<FootstepDataMessage> goals)
   {
      FootstepPlanRequestPacket message = new FootstepPlanRequestPacket();
      message.footstepPlanRequestType = requestType.toByte();
      message.startFootstep = startFootstep;
      message.thetaStart = thetaStart;
      message.goals = goals;
      return message;
   }

   public static FootstepPlanRequestPacket createFootstepPlanRequestPacket(FootstepPlanRequestType requestType, FootstepDataMessage startFootstep, double thetaStart,
                                                                           ArrayList<FootstepDataMessage> goals, double maxSuboptimality)
   {
      FootstepPlanRequestPacket message = new FootstepPlanRequestPacket();
      message.footstepPlanRequestType = requestType.toByte();
      message.startFootstep = startFootstep;
      message.thetaStart = thetaStart;
      message.goals = goals;
      message.maxSuboptimality = maxSuboptimality;
      return message;
   }

   public static HandJointAnglePacket createHandJointAnglePacket(RobotSide robotSide, boolean connected, boolean calibrated, double[] jointAngles)
   {
      HandJointAnglePacket message = new HandJointAnglePacket();
      message.robotSide = robotSide.toByte();
      if (message.jointAngles == null)
      {
         message.jointAngles = new double[jointAngles.length];
      }
      System.arraycopy(jointAngles, 0, message.jointAngles, 0, jointAngles.length);
      message.connected = connected;
      message.calibrated = calibrated;
      return message;
   }

   public static HandCollisionDetectedPacket createHandCollisionDetectedPacket(RobotSide robotSide, int collisionSeverityLevelZeroToThree)
   {
      HandCollisionDetectedPacket message = new HandCollisionDetectedPacket();
      message.robotSide = robotSide.toByte();
      message.collisionSeverityLevelOneToThree = MathTools.clamp(collisionSeverityLevelZeroToThree, 1, 3);
      return message;
   }

   public static AtlasLowLevelControlModeMessage createAtlasLowLevelControlModeMessage(AtlasLowLevelControlMode request)
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.requestedAtlasLowLevelControlMode = request.toByte();
      return message;
   }

   public static HandLoadBearingMessage createHandLoadBearingMessage(RobotSide robotSide)
   {
      HandLoadBearingMessage message = new HandLoadBearingMessage();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static BehaviorControlModeResponsePacket createBehaviorControlModeResponsePacket(BehaviorControlModeEnum requestedControl)
   {
      BehaviorControlModeResponsePacket message = new BehaviorControlModeResponsePacket();
      message.behaviorControlModeEnumRequest = requestedControl.toByte();
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage(OneDoFTrajectoryPointInterface<?> trajectoryPoint)
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.time = trajectoryPoint.getTime();
      message.position = trajectoryPoint.getPosition();
      message.velocity = trajectoryPoint.getVelocity();
      return message;
   }

   public static TrajectoryPoint1DMessage createTrajectoryPoint1DMessage(double time, double position, double velocity)
   {
      TrajectoryPoint1DMessage message = new TrajectoryPoint1DMessage();
      message.time = time;
      message.position = position;
      message.velocity = velocity;
      return message;
   }

   public static StateEstimatorModePacket createStateEstimatorModePacket(StateEstimatorMode requestedOperatingMode)
   {
      StateEstimatorModePacket message = new StateEstimatorModePacket();
      message.requestedStateEstimatorMode = requestedOperatingMode.toByte();
      return message;
   }

   public static HumanoidBehaviorTypePacket createHumanoidBehaviorTypePacket(HumanoidBehaviorType behaviorType)
   {
      HumanoidBehaviorTypePacket message = new HumanoidBehaviorTypePacket();
      message.humanoidBehaviorType = behaviorType.toByte();
      return message;
   }

   public static FootstepDataListMessage createFootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double finalTransferDuration)
   {
      return createFootstepDataListMessage(footstepDataList, 0.0, 0.0, finalTransferDuration, ExecutionMode.OVERRIDE);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double defaultSwingDuration,
                                                                       double defaultTransferDuration, ExecutionMode executionMode)
   {
      return createFootstepDataListMessage(footstepDataList, defaultSwingDuration, defaultTransferDuration, defaultTransferDuration, executionMode);
   }

   /**
    *
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(ArrayList<FootstepDataMessage> footstepDataList, double defaultSwingDuration,
                                                                       double defaultTransferDuration, double finalTransferDuration,
                                                                       ExecutionMode executionMode)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      if (footstepDataList != null)
      {
         message.footstepDataList = footstepDataList;
      }
      message.defaultSwingDuration = defaultSwingDuration;
      message.defaultTransferDuration = defaultTransferDuration;
      message.finalTransferDuration = finalTransferDuration;
      message.queueingProperties.setExecutionMode(executionMode.toByte());
      message.queueingProperties.setPreviousMessageId(Packet.VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   /**
    *
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
    *
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
      message.defaultSwingDuration = defaultSwingDuration;
      message.defaultTransferDuration = defaultTransferDuration;
      message.finalTransferDuration = finalTransferDuration;
      message.queueingProperties.setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      message.queueingProperties.setPreviousMessageId(Packet.VALID_MESSAGE_DEFAULT_ID);
      return message;
   }

   /**
    * Creates a message with the desired grasp to be performed. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide refers to which hand will perform the grasp.
    * @param handDesiredConfiguration refers to the desired grasp.
    */
   public static HandDesiredConfigurationMessage createHandDesiredConfigurationMessage(RobotSide robotSide, HandConfiguration handDesiredConfiguration)
   {
      HandDesiredConfigurationMessage message = new HandDesiredConfigurationMessage();
      message.robotSide = robotSide.toByte();
      message.desiredHandConfiguration = handDesiredConfiguration.toByte();
      return message;
   }

   public static FisheyePacket createFisheyePacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position,
                                                   QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      FisheyePacket message = new FisheyePacket();
      message.videoSource = videoSource.toByte();
      message.timeStamp = timeStamp;
      message.data = data;
      message.position = new Point3D(position);
      message.orientation = new Quaternion(orientation);
      message.intrinsicParameters = intrinsicParameters;
      return message;
   }

   public static VideoPacket createVideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation,
                                               IntrinsicParameters intrinsicParameters)
   {
      return createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters, null);
   }

   public static VideoPacket createVideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation,
                                               IntrinsicParameters intrinsicParameters, PacketDestination packetDestination)
   {
      VideoPacket message = new VideoPacket();
      if (packetDestination != null)
         message.setDestination(packetDestination);
      message.videoSource = videoSource.toByte();
      message.timeStamp = timeStamp;
      message.data = data;
      message.position = new Point3D(position);
      message.orientation = new Quaternion(orientation);
      message.intrinsicParameters = intrinsicParameters;
      return message;
   }

   public static LocalVideoPacket createLocalVideoPacket(long timeStamp, BufferedImage image, IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket message = new LocalVideoPacket();
      message.timeStamp = timeStamp;
      message.image = image;
      message.intrinsicParameters = intrinsicParameters;
      return message;
   }

   /**
    * 
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param pause
    */
   public static PauseWalkingMessage createPauseWalkingMessage(boolean pause)
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.pause = pause;
      return message;
   }

   public static ReachingManifoldMessage createReachingManifoldMessage(RigidBody rigidBody)
   {
      ReachingManifoldMessage message = new ReachingManifoldMessage();
      message.endEffectorNameBasedHashCode = rigidBody.getNameBasedHashCode();
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
      message.numberOfInitialGuesses = numberOfInitialGuesses;
      message.maximumExpansionSize = maximumExpansionSize;
      return message;
   }

   public static SO3TrajectoryPointMessage createSO3TrajectoryPointMessage(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      SO3TrajectoryPointMessage message = new SO3TrajectoryPointMessage();
      message.time = time;
      message.orientation = new Quaternion(orientation);
      message.angularVelocity = new Vector3D(angularVelocity);
      return message;
   }

   public static ArmDesiredAccelerationsMessage createArmDesiredAccelerationsMessage(RobotSide robotSide, double[] armDesiredJointAccelerations)
   {
      ArmDesiredAccelerationsMessage message = new ArmDesiredAccelerationsMessage();
      message.desiredAccelerations = HumanoidMessageTools.createDesiredAccelerationsMessage(armDesiredJointAccelerations);
      message.robotSide = robotSide.toByte();
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
      message.desiredAccelerations = HumanoidMessageTools.createDesiredAccelerationsMessage(desiredJointAccelerations);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of controlled joints.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      message.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < message.getNumberOfJoints(); jointIndex++)
         message.jointTrajectoryMessages[jointIndex] = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *           number of controlled joints.
    * @param weights the qp weights for the joint accelerations
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      message.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < message.getNumberOfJoints(); jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
         oneDoFJointTrajectoryMessage.setWeight(weights[jointIndex]);
         message.jointTrajectoryMessages[jointIndex] = oneDoFJointTrajectoryMessage;
      }
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(int numberOfJoints)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      message.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[numberOfJoints];
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory points. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory
    * point afterwards. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      message.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         message.jointTrajectoryMessages[i] = createOneDoFJointTrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      JointspaceTrajectoryMessage message = new JointspaceTrajectoryMessage();
      message.jointTrajectoryMessages = jointTrajectory1DListMessages;
      return message;
   }

   public static SimpleCoactiveBehaviorDataPacket createSimpleCoactiveBehaviorDataPacket(String key, double value)
   {
      SimpleCoactiveBehaviorDataPacket message = new SimpleCoactiveBehaviorDataPacket();
      message.key = key;
      message.value = value;
      return message;
   }

   public static BDIBehaviorCommandPacket createBDIBehaviorCommandPacket(boolean stop)
   {
      BDIBehaviorCommandPacket message = new BDIBehaviorCommandPacket();
      message.stop = stop;
      return message;
   }

   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(SimpleTrajectoryPoint1DList trajectoryData)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      int numberOfPoints = trajectoryData.getNumberOfTrajectoryPoints();
      message.trajectoryPoints = new TrajectoryPoint1DMessage[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(i);
         message.trajectoryPoints[i] = HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint);
      }
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point.
    * 
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      message.trajectoryPoints = new TrajectoryPoint1DMessage[] {HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryTime, desiredPosition, 0.0)};
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point.
    * 
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    * @param weight the weight for the qp
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition, double weight)
   {
      OneDoFJointTrajectoryMessage message = createOneDoFJointTrajectoryMessage(trajectoryTime, desiredPosition);
      message.weight = weight;
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory points. This constructor
    * only allocates memory for the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      message.trajectoryPoints = new TrajectoryPoint1DMessage[numberOfTrajectoryPoints];
      return message;
   }

   public static AtlasDesiredPumpPSIPacket createAtlasDesiredPumpPSIPacket(int desiredPumpPsi)
   {
      AtlasDesiredPumpPSIPacket message = new AtlasDesiredPumpPSIPacket();
      message.desiredPumpPsi = desiredPumpPsi;
      return message;
   }

   public static AtlasElectricMotorAutoEnableFlagPacket createAtlasElectricMotorAutoEnableFlagPacket(boolean shouldAutoEnable)
   {
      AtlasElectricMotorAutoEnableFlagPacket message = new AtlasElectricMotorAutoEnableFlagPacket();
      message.shouldAutoEnable = shouldAutoEnable;
      return message;
   }

   public static EuclideanTrajectoryPointMessage createEuclideanTrajectoryPointMessage(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      EuclideanTrajectoryPointMessage message = new EuclideanTrajectoryPointMessage();
      message.time = time;
      message.position = new Point3D(position);
      message.linearVelocity = new Vector3D(linearVelocity);
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket(WalkToGoalAction action)
   {
      WalkToGoalBehaviorPacket message = new WalkToGoalBehaviorPacket();
      message.walkToGoalAction = action.toByte();
      return message;
   }

   public static WalkToGoalBehaviorPacket createWalkToGoalBehaviorPacket(double xGoal, double yGoal, double thetaGoal, RobotSide goalSide)
   {
      WalkToGoalBehaviorPacket message = new WalkToGoalBehaviorPacket();
      message.walkToGoalAction = WalkToGoalAction.FIND_PATH.toByte();
      message.xGoal = xGoal;
      message.yGoal = yGoal;
      message.thetaGoal = thetaGoal;
      message.goalRobotSide = goalSide.toByte();
      return message;
   }

   public static BDIBehaviorStatusPacket createBDIBehaviorStatusPacket(BDIRobotBehavior currentBehavior)
   {
      BDIBehaviorStatusPacket message = new BDIBehaviorStatusPacket();
      message.currentBDIRobotBehavior = currentBehavior.toByte();
      return message;
   }

   public static CenterOfMassTrajectoryMessage createCenterOfMassTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      CenterOfMassTrajectoryMessage message = new CenterOfMassTrajectoryMessage();
      message.euclideanTrajectory = HumanoidMessageTools.createEuclideanTrajectoryMessage(numberOfTrajectoryPoints);
      return message;
   }

   public static SpineTrajectoryMessage createSpineTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      message.setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory points. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory
    * point afterwards. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.jointspaceTrajectory = HumanoidMessageTools.createJointspaceTrajectoryMessage(numberOfJoints, numberOfTrajectoryPoints);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectories, you need to call
    * {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(int numberOfJoints)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.jointspaceTrajectory = HumanoidMessageTools.createJointspaceTrajectoryMessage(numberOfJoints);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number
    *           of joints.
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.jointspaceTrajectory = HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, jointDesireds);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number
    *           of joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that
    *           joint will use the controller default weight
    */
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds, double[] weights)
   {
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      message.jointspaceTrajectory = HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTime, jointDesireds, weights);
      return message;
   }

   public static AutomaticManipulationAbortMessage createAutomaticManipulationAbortMessage(boolean enable)
   {
      AutomaticManipulationAbortMessage message = new AutomaticManipulationAbortMessage();
      message.enable = enable;
      return message;
   }

   public static StampedPosePacket createStampedPosePacket(String frameId, TimeStampedTransform3D transform, double confidenceFactor)
   {
      StampedPosePacket message = new StampedPosePacket();
      message.frameId = frameId;
      message.transform = transform;
      message.confidenceFactor = confidenceFactor;
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 long trajectoryReferenceFrameId)
   {
      SE3TrajectoryMessage message = new SE3TrajectoryMessage();
      Vector3D zeroLinearVelocity = new Vector3D();
      Vector3D zeroAngularVelocity = new Vector3D();
      message.taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {
            createSE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
      message.frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame.getNameBasedHashCode());
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(int numberOfTrajectoryPoints)
   {
      SE3TrajectoryMessage message = new SE3TrajectoryMessage();
      message.taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfTrajectoryPoints];
      return message;
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(int numberOfPoints, ReferenceFrame trajectoryFrame)
   {
      SE3TrajectoryMessage message = createSE3TrajectoryMessage(numberOfPoints);
      message.getFrameInformation().setTrajectoryReferenceFrame(trajectoryFrame);
      return message;
   }

   public static DetectedObjectPacket createDetectedObjectPacket(RigidBodyTransform pose, int id)
   {
      DetectedObjectPacket message = new DetectedObjectPacket();
      message.pose = pose;
      message.id = id;
      return message;
   }

   public static WalkingControllerFailureStatusMessage createWalkingControllerFailureStatusMessage(Vector2D fallingDirection)
   {
      WalkingControllerFailureStatusMessage message = new WalkingControllerFailureStatusMessage();
      message.fallingDirection = new Vector2D32(fallingDirection);
      return message;
   }

   public static AtlasWristSensorCalibrationRequestPacket createAtlasWristSensorCalibrationRequestPacket(RobotSide robotSide)
   {
      AtlasWristSensorCalibrationRequestPacket message = new AtlasWristSensorCalibrationRequestPacket();
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static GoHomeMessage createGoHomeMessage(HumanoidBodyPart bodyPart, double trajectoryTime)
   {
      GoHomeMessage message = new GoHomeMessage();
      HumanoidMessageTools.checkRobotSide(bodyPart);
      message.humanoidBodyPart = bodyPart.toByte();
      message.trajectoryTime = trajectoryTime;
      return message;
   }

   public static GoHomeMessage createGoHomeMessage(HumanoidBodyPart bodyPart, RobotSide robotSide, double trajectoryTime)
   {
      GoHomeMessage message = new GoHomeMessage();
      if (robotSide == null)
         HumanoidMessageTools.checkRobotSide(bodyPart);
      message.humanoidBodyPart = bodyPart.toByte();
      message.robotSide = robotSide.toByte();
      message.trajectoryTime = trajectoryTime;
      return message;
   }

   public static SE3TrajectoryPointMessage createSE3TrajectoryPointMessage(double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                                                           Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      SE3TrajectoryPointMessage message = new SE3TrajectoryPointMessage();
      message.time = time;
      message.position = new Point3D(position);
      message.orientation = new Quaternion(orientation);
      message.linearVelocity = new Vector3D(linearVelocity);
      message.angularVelocity = new Vector3D(angularVelocity);
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, QuaternionReadOnly orientation)
   {
      return createFootstepDataMessage(robotSide, new Point3D(location), new Quaternion(orientation), null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                               ArrayList<Point2D> predictedContactPoints)
   {
      return createFootstepDataMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation, TrajectoryType trajectoryType,
                                                               double swingHeight)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                               ArrayList<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      FootstepDataMessage message = new FootstepDataMessage();
      message.robotSide = robotSide.toByte();
      message.location = location;
      message.orientation = orientation;
      if (predictedContactPoints != null && predictedContactPoints.size() == 0)
         message.predictedContactPoints = null;
      else
         message.predictedContactPoints = predictedContactPoints;
      message.trajectoryType = trajectoryType.toByte();
      message.swingHeight = swingHeight;
      return message;
   }

   public static FootstepDataMessage createFootstepDataMessage(Footstep footstep)
   {
      FootstepDataMessage message = new FootstepDataMessage();

      message.robotSide = footstep.getRobotSide().toByte();

      FramePoint3D location = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(location, orientation);
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      message.location = new Point3D(location);
      message.orientation = new Quaternion(orientation);

      List<Point2D> footstepContactPoints = footstep.getPredictedContactPoints();
      if (footstepContactPoints != null)
      {
         if (message.predictedContactPoints == null)
         {
            message.predictedContactPoints = new ArrayList<>();
         }
         else
         {
            message.predictedContactPoints.clear();
         }
         for (int i = 0; i < footstepContactPoints.size(); i++)
         {
            Point2D point = new Point2D(footstepContactPoints.get(i));
            message.predictedContactPoints.add(point);
         }
      }
      else
      {
         message.predictedContactPoints = null;
      }
      message.trajectoryType = footstep.getTrajectoryType().toByte();
      message.swingHeight = footstep.getSwingHeight();
      message.swingTrajectoryBlendDuration = footstep.getSwingTrajectoryBlendDuration();

      if (footstep.getCustomPositionWaypoints().size() != 0)
      {
         message.positionWaypoints = new Point3D[footstep.getCustomPositionWaypoints().size()];
         for (int i = 0; i < footstep.getCustomPositionWaypoints().size(); i++)
         {
            FramePoint3D framePoint = footstep.getCustomPositionWaypoints().get(i);
            framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
            message.positionWaypoints[i] = new Point3D(framePoint);
         }
      }

      return message;
   }

   public static PlanOffsetStatus createPlanOffsetStatus(Vector3DReadOnly offsetVector)
   {
      PlanOffsetStatus message = new PlanOffsetStatus();
      message.offsetVector.set(offsetVector);
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
      message.clazz = clazz;
      return message;
   }

   public static LocalizationStatusPacket createLocalizationStatusPacket(double overlap, String status)
   {
      LocalizationStatusPacket message = new LocalizationStatusPacket();
      message.overlap = overlap;
      message.status = status;
      return message;
   }

   public static ValveLocationPacket createValveLocationPacket(RigidBodyTransform valveTransformToWorld, double valveRadius)
   {
      ValveLocationPacket message = new ValveLocationPacket();
      message.valveTransformToWorld = valveTransformToWorld;
      message.valveRadius = valveRadius;
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.se3Trajectory = new SE3TrajectoryMessage(trajectoryMessage);
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as
    * the base for the control. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition,
                                                                   QuaternionReadOnly desiredOrientation)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.se3Trajectory = HumanoidMessageTools.createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation,
                                                                              ReferenceFrame.getWorldFrame());
      message.robotSide = robotSide.toByte();
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Point3D, Quaternion, Vector3D, Vector3D)} for each
    * trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.se3Trajectory = HumanoidMessageTools.createSE3TrajectoryMessage(numberOfTrajectoryPoints);
      message.robotSide = robotSide.toByte();
      return message;
   }

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, FramePose3D desiredPose)
   {
      return createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation());
   }

   public static PrepareForLocomotionMessage createPrepareForLocomotionMessage(boolean prepareManipulation, boolean preparePelvis)
   {
      PrepareForLocomotionMessage message = new PrepareForLocomotionMessage();
      message.prepareManipulation = prepareManipulation;
      message.preparePelvis = preparePelvis;
      return message;
   }

   public static WalkOverTerrainGoalPacket createWalkOverTerrainGoalPacket(Point3D position, Quaternion orientation)
   {
      WalkOverTerrainGoalPacket message = new WalkOverTerrainGoalPacket();
      message.position = position;
      message.orientation = orientation;
      return message;
   }

   public static void checkRobotSide(HumanoidBodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
   }
}
