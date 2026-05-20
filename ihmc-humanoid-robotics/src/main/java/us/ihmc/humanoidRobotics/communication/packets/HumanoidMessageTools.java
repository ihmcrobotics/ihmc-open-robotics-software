package us.ihmc.humanoidRobotics.communication.packets;

import controller_msgs.ArmTrajectoryMessage;
import controller_msgs.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.DesiredAccelerationsMessage;
import controller_msgs.FootLoadBearingMessage;
import controller_msgs.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.HandTrajectoryMessage;
import controller_msgs.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.HighLevelStateChangeStatusMessage;
import controller_msgs.JointspaceTrajectoryMessage;
import controller_msgs.NeckDesiredAccelerationsMessage;
import controller_msgs.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.SE3TrajectoryMessage;
import ihmc_common_msgs.SO3TrajectoryMessage;
import toolbox_msgs.RigidBodyExplorationConfigurationMessage;
import toolbox_msgs.WaypointBasedTrajectoryMessage;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.partNames.HandJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static us.ihmc.euclid.tools.EuclidCoreTools.zeroVector3D;

public class HumanoidMessageTools
{
   public static final int CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES = 8;

   private HumanoidMessageTools()
   {
   }

   public static DesiredAccelerationsMessage createDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      DesiredAccelerationsMessage message = new DesiredAccelerationsMessage();
      message.getDesiredJointAccelerations().addAll(desiredJointAccelerations);
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
      JointspaceTrajectoryMessage copy = new JointspaceTrajectoryMessage();
      copy.set(jointspaceTrajectoryMessage);
      message.getJointspaceTrajectory().set(copy);
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
   public static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide robotSide,
                                                                 double trajectoryTime,
                                                                 double[] desiredJointPositions,
                                                                 double[] desiredJointVelocities,
                                                                 double[] weights)
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
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Point3DReadOnly desiredPosition,
                                                                   Orientation3DReadOnly desiredOrientation,
                                                                   long trajectoryReferenceFrameId)
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
   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Point3DReadOnly desiredPosition,
                                                                   Orientation3DReadOnly desiredOrientation,
                                                                   ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Pose3DReadOnly desiredPose,
                                                                   ReferenceFrame trajectoryReferenceFrame)
   {
      return createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPose, MessageTools.toFrameId(trajectoryReferenceFrame));
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Pose3DReadOnly desiredPose,
                                                                   long trajectoryReferenceFrameID)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPose, trajectoryReferenceFrameID));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   public static HandTrajectoryMessage createHandTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Pose3DReadOnly desiredPose,
                                                                   SpatialVectorReadOnly desiredVelocity,
                                                                   ReferenceFrame trajectoryReferenceFrame)
   {
      HandTrajectoryMessage message = new HandTrajectoryMessage();
      message.getSe3Trajectory().set(createSE3TrajectoryMessage(trajectoryTime, desiredPose, desiredVelocity, trajectoryReferenceFrame));
      message.setRobotSide(robotSide.toByte());
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

      message.getConfigurationSpaceNamesToExplore().getBuffer().reset();
      message.getExplorationRangeUpperLimits().clear();
      message.getExplorationRangeLowerLimits().clear();

      message.getConfigurationSpaceNamesToExplore().addAll(degreesOfFreedomToExplore1);

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

      message.getConfigurationSpaceNamesToExplore().getBuffer().reset();
      message.getExplorationRangeUpperLimits().clear();
      message.getExplorationRangeLowerLimits().clear();

      message.getConfigurationSpaceNamesToExplore().addAll(degreesOfFreedomToExplore1);
      message.getExplorationRangeUpperLimits().addAll(explorationRangeUpperLimits);
      message.getExplorationRangeLowerLimits().addAll(explorationRangeLowerLimits);

      return message;
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBodyBasics endEffector, double[] waypointTimes, Pose3D[] waypoints)
   {
      return createWaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints);
   }

   public static WaypointBasedTrajectoryMessage createWaypointBasedTrajectoryMessage(RigidBodyBasics endEffector,
                                                                                     double[] waypointTimes,
                                                                                     Pose3D[] waypoints,
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
   public static PelvisTrajectoryMessage createPelvisTrajectoryMessage(double trajectoryTime,
                                                                       Point3DReadOnly desiredPosition,
                                                                       Orientation3DReadOnly desiredOrientation)
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
   public static NeckTrajectoryMessage createNeckTrajectoryMessage(double trajectoryTime,
                                                                   double[] desiredJointPositions,
                                                                   double[] desiredJointVelocities,
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
   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime, Orientation3DReadOnly desiredOrientation)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, ReferenceFrame.getWorldFrame()));
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime,
                                                                                             Orientation3DReadOnly desiredOrientation,
                                                                                             ReferenceFrame trajectoryFrame)
   {
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryFrame));
      return message;
   }

   public static PelvisOrientationTrajectoryMessage createPelvisOrientationTrajectoryMessage(double trajectoryTime,
                                                                                             Orientation3DReadOnly desiredOrientation,
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

   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime,
                                                                     Orientation3DReadOnly desiredOrientation)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, ReferenceFrame.getWorldFrame()));
      return message;
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    *
    * @param trajectoryTime     how long it takes to reach the desired orientation.
    * @param desiredOrientation desired chest orientation expressed in World.
    */
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime,
                                                                     Orientation3DReadOnly desiredOrientation,
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
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime,
                                                                     Orientation3DReadOnly desiredOrientation,
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
   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime,
                                                                     Orientation3DReadOnly desiredOrientation,
                                                                     Vector3DReadOnly desiredAngularVelocity,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame));
      return message;
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime,
                                                                     Orientation3DReadOnly desiredOrientation,
                                                                     ReferenceFrame dataFrame,
                                                                     ReferenceFrame trajectoryFrame)
   {
      return createChestTrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, dataFrame, trajectoryFrame);
   }

   public static ChestTrajectoryMessage createChestTrajectoryMessage(double trajectoryTime,
                                                                     Orientation3DReadOnly desiredOrientation,
                                                                     Vector3DReadOnly desiredAngularVelocity,
                                                                     ReferenceFrame dataFrame,
                                                                     ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage message = createChestTrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame);
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataFrame));
      return message;
   }

   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime,
                                                                   Orientation3DReadOnly desiredOrientation,
                                                                   ReferenceFrame dataFrame,
                                                                   ReferenceFrame trajectoryFrame)
   {
      return createHeadTrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, dataFrame, trajectoryFrame);
   }

   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime,
                                                                   Orientation3DReadOnly desiredOrientation,
                                                                   Vector3DReadOnly desiredAngularVelocity,
                                                                   ReferenceFrame dataFrame,
                                                                   ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame));
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
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime,
                                                                   Orientation3DReadOnly desiredOrientation,
                                                                   ReferenceFrame trajectoryFrame)
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
   public static HeadTrajectoryMessage createHeadTrajectoryMessage(double trajectoryTime,
                                                                   Orientation3DReadOnly desiredOrientation,
                                                                   long trajectoryReferenceFrameId)
   {
      HeadTrajectoryMessage message = new HeadTrajectoryMessage();
      message.getSo3Trajectory().set(createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, zeroVector3D, trajectoryReferenceFrameId));
      return message;
   }

   /**
    * Generates and publishes the head hybrid trajectory (SO3 + Jointspace).
    * Used for both UI widgets and N-pose.
    *
    * @param neckJointNamesArray The array of the neck joint names
    * @param desiredNeckPositions Array of desired joint positions (length = neck joints).
    * @param trajectoryTime Duration for the trajectory.
    */
   public static HeadHybridJointspaceTaskspaceTrajectoryMessage createHeadJointspaceTaskspaceTrajectoryMessage(HumanoidReferenceFrames referenceFrames,
                                                                                                               NeckJointName[] neckJointNamesArray,
                                                                                                               double[] desiredNeckPositions,
                                                                                                               double trajectoryTime)
   {
      FrameYawPitchRoll frameHeadYawPitchRoll = new FrameYawPitchRoll();

      // Compose the yaw/pitch/roll from desiredNeckPositions
      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         switch (neckJointNamesArray[i])
         {
            case PROXIMAL_NECK_PITCH ->
            {
               frameHeadYawPitchRoll.changeFrame(referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH));
               frameHeadYawPitchRoll.setToZero();
               frameHeadYawPitchRoll.setPitch(desiredNeckPositions[i]);
            }
            case PROXIMAL_NECK_YAW ->
            {
               frameHeadYawPitchRoll.changeFrame(referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_YAW));
               frameHeadYawPitchRoll.setToZero();
               frameHeadYawPitchRoll.setYaw(desiredNeckPositions[i]);
            }
            case PROXIMAL_NECK_ROLL ->
            {
               frameHeadYawPitchRoll.changeFrame(referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH));
               frameHeadYawPitchRoll.setToZero();
               frameHeadYawPitchRoll.setRoll(desiredNeckPositions[i]);
            }
            case DISTAL_NECK_PITCH ->
            {
               frameHeadYawPitchRoll.changeFrame(referenceFrames.getNeckFrame(NeckJointName.DISTAL_NECK_PITCH));
               frameHeadYawPitchRoll.setToZero();
               frameHeadYawPitchRoll.setPitch(desiredNeckPositions[i]);
            }
            case DISTAL_NECK_YAW ->
            {
               frameHeadYawPitchRoll.changeFrame(referenceFrames.getNeckFrame(NeckJointName.DISTAL_NECK_YAW));
               frameHeadYawPitchRoll.setToZero();
               frameHeadYawPitchRoll.setYaw(desiredNeckPositions[i]);
            }
            case DISTAL_NECK_ROLL ->
            {
               frameHeadYawPitchRoll.changeFrame(referenceFrames.getNeckFrame(NeckJointName.DISTAL_NECK_ROLL));
               frameHeadYawPitchRoll.setToZero();
               frameHeadYawPitchRoll.setRoll(desiredNeckPositions[i]);
            }
         }
      }

      frameHeadYawPitchRoll.changeFrame(referenceFrames.getChestFrame());
      SO3TrajectoryMessage taskspaceTrajectoryMessage = HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime,
                                                                                                        frameHeadYawPitchRoll,
                                                                                                        EuclidCoreTools.zeroVector3D,
                                                                                                        referenceFrames.getChestFrame());

      taskspaceTrajectoryMessage.getWeightMatrix().setXWeight(0.01);
      taskspaceTrajectoryMessage.getWeightMatrix().setYWeight(0.01);
      taskspaceTrajectoryMessage.getWeightMatrix().setZWeight(0.01);

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildHeadJointspaceTrajectoryMessage(desiredNeckPositions, trajectoryTime);

      HeadHybridJointspaceTaskspaceTrajectoryMessage hybridMessage = new HeadHybridJointspaceTaskspaceTrajectoryMessage();
      hybridMessage.getTaskspaceTrajectoryMessage().set(taskspaceTrajectoryMessage);
      hybridMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);

      return hybridMessage;
   }

   /**
    * Build JointspaceTrajectoryMessage for head, parametrized.
    */
   private static JointspaceTrajectoryMessage buildHeadJointspaceTrajectoryMessage(double[] jointAngles,
                                                                                   double trajectoryTime)
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      for (double q : jointAngles)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         oneDoFJointTrajectoryMessage.setWeight(100.0);
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(trajectoryTime);
         trajectoryPoint1DMessage.setPosition(q);
         trajectoryPoint1DMessage.setVelocity(0.0);
      }

      return jointspaceTrajectoryMessage;
   }

   /**
    * set a single point
    *
    * @param trajectoryTime             the duration of the trajectory
    * @param desiredPosition            the desired end position
    * @param trajectoryReferenceFrameId the frame id the trajectory will be executed in
    */
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime,
                                                                             Point3DReadOnly desiredPosition,
                                                                             long trajectoryReferenceFrameId)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, zeroVector3D, trajectoryReferenceFrameId);
   }

   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime,
                                                                             Point3DReadOnly desiredPosition,
                                                                             Vector3DReadOnly desiredLinearVelocity,
                                                                             long trajectoryReferenceFrameId)
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
   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime,
                                                                             Point3DReadOnly desiredPosition,
                                                                             ReferenceFrame trajectoryReferenceFrame)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, zeroVector3D, trajectoryReferenceFrame);
   }

   public static EuclideanTrajectoryMessage createEuclideanTrajectoryMessage(double trajectoryTime,
                                                                             Point3DReadOnly desiredPosition,
                                                                             Vector3DReadOnly desiredLinearVelocity,
                                                                             ReferenceFrame trajectoryReferenceFrame)
   {
      return createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, desiredLinearVelocity, trajectoryReferenceFrame.getFrameNameHashCode());
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
   public static PelvisHeightTrajectoryMessage createPelvisHeightTrajectoryMessage(double trajectoryTime,
                                                                                   double desiredHeight,
                                                                                   ReferenceFrame trajectoryReferenceFrame,
                                                                                   ReferenceFrame dataReferenceFrame)
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                        new Point3D(0.0, 0.0, desiredHeight),
                                                                        trajectoryReferenceFrame.getFrameNameHashCode()));
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

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status,
                                                            int footstepIndex,
                                                            Point3D actualFootPositionInWorld,
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

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status,
                                                            int footstepIndex,
                                                            Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld,
                                                            RobotSide robotSide)
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

   public static FootstepStatusMessage createFootstepStatus(FootstepStatus status,
                                                            int footstepIndex,
                                                            Point3D desiredFootPositionInWorld,
                                                            Quaternion desiredFootOrientationInWorld,
                                                            Point3D actualFootPositionInWorld,
                                                            Quaternion actualFootOrientationInWorld,
                                                            RobotSide robotSide)
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

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime,
                                                                 Orientation3DReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredAngularVelocity,
                                                                 ReferenceFrame trajectoryFrame)
   {
      return createSO3TrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, trajectoryFrame.getFrameNameHashCode());
   }

   public static SO3TrajectoryMessage createSO3TrajectoryMessage(double trajectoryTime,
                                                                 Orientation3DReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredAngularVelocity,
                                                                 long trajectoryReferenceFrameId)
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

   public static HandJointAnglePacket createHandJointAnglePacket(RobotSide robotSide, boolean connected, boolean calibrated, double[] jointAngles)
   {
      HandJointAnglePacket message = new HandJointAnglePacket();
      message.setRobotSide(robotSide == null ? -1 : robotSide.toByte());
      message.getJointAngles().add(jointAngles);
      message.setConnected(connected);
      message.setCalibrated(calibrated);
      return message;
   }

   public static HandLoadBearingMessage createHandLoadBearingMessage(RobotSide robotSide)
   {
      HandLoadBearingMessage message = new HandLoadBearingMessage();
      message.setRobotSide(robotSide.toByte());
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

   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList, double finalTransferDuration)
   {
      return createFootstepDataListMessage(footstepDataList, 0.0, 0.0, finalTransferDuration, ExecutionMode.OVERRIDE);
   }

   public static FootstepDataListMessage createFootstepDataListMessage(FootstepDataMessage... footstepDataList)
   {
      List<FootstepDataMessage> messageList = new ArrayList<>();
      for (FootstepDataMessage message : footstepDataList)
         messageList.add(message);

      return createFootstepDataListMessage(messageList, -1.0);
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList,
                                                                       double defaultSwingDuration,
                                                                       double defaultTransferDuration,
                                                                       ExecutionMode executionMode)
   {
      return createFootstepDataListMessage(footstepDataList, defaultSwingDuration, defaultTransferDuration, defaultTransferDuration, executionMode);
   }

   /**
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param footstepDataList
    * @param defaultSwingDuration
    * @param defaultTransferDuration
    * @param finalTransferDuration
    * @param executionMode
    */
   public static FootstepDataListMessage createFootstepDataListMessage(List<FootstepDataMessage> footstepDataList,
                                                                       double defaultSwingDuration,
                                                                       double defaultTransferDuration,
                                                                       double finalTransferDuration,
                                                                       ExecutionMode executionMode)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      MessageTools.copyData(footstepDataList, message.getFootstepDataList());
      message.setDefaultSwingDuration(defaultSwingDuration);
      message.setDefaultTransferDuration(defaultTransferDuration);
      message.setFinalTransferDuration(finalTransferDuration);
      message.getQueueingProperties().setExecutionMode(executionMode.toByte());
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
   public static FootstepDataListMessage createFootstepDataListMessage(double defaultSwingDuration,
                                                                       double defaultTransferDuration,
                                                                       double finalTransferDuration)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      message.setDefaultSwingDuration(defaultSwingDuration);
      message.setDefaultTransferDuration(defaultTransferDuration);
      message.setFinalTransferDuration(finalTransferDuration);
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

   public static SO3TrajectoryPointMessage createSO3TrajectoryPointMessage(double time, Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity)
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
   public static JointspaceTrajectoryMessage createJointspaceTrajectoryMessage(double trajectoryTime,
                                                                               double[] desiredJointPositions,
                                                                               double[] desiredJointVelocities,
                                                                               double[] weights)
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
   public static OneDoFJointTrajectoryMessage createOneDoFJointTrajectoryMessage(double trajectoryTime,
                                                                                 double desiredPosition,
                                                                                 double desiredVelocity,
                                                                                 double weight)
   {
      OneDoFJointTrajectoryMessage message = new OneDoFJointTrajectoryMessage();
      message.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryTime, desiredPosition, desiredVelocity));
      message.setWeight(weight);
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
   public static SpineTrajectoryMessage createSpineTrajectoryMessage(double trajectoryTime,
                                                                     double[] desiredJointPositions,
                                                                     double[] desiredJointVelocities,
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

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime, Pose3DReadOnly desiredPose, long trajectoryReferenceFrameId)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation(), trajectoryReferenceFrameId);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime,
                                                                 Point3DReadOnly desiredPosition,
                                                                 Orientation3DReadOnly desiredOrientation,
                                                                 long trajectoryReferenceFrameId)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroVector3D, zeroVector3D, trajectoryReferenceFrameId);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime,
                                                                 Point3DReadOnly desiredPosition,
                                                                 Orientation3DReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredLinearVelocity,
                                                                 Vector3DReadOnly desiredAngularVelocity,
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

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime,
                                                                 Pose3DReadOnly desiredPose,
                                                                 SpatialVectorReadOnly desiredVelocity,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime,
                                        desiredPose.getPosition(),
                                        desiredPose.getOrientation(),
                                        desiredVelocity.getLinearPart(),
                                        desiredVelocity.getAngularPart(),
                                        trajectoryReferenceFrame);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime,
                                                                 Point3DReadOnly desiredPosition,
                                                                 Orientation3DReadOnly desiredOrientation,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroVector3D, zeroVector3D, trajectoryReferenceFrame);
   }

   public static SE3TrajectoryMessage createSE3TrajectoryMessage(double trajectoryTime,
                                                                 Point3DReadOnly desiredPosition,
                                                                 Orientation3DReadOnly desiredOrientation,
                                                                 Vector3DReadOnly desiredLinearVelocity,
                                                                 Vector3DReadOnly desiredAngularVelocity,
                                                                 ReferenceFrame trajectoryReferenceFrame)
   {
      return createSE3TrajectoryMessage(trajectoryTime,
                                        desiredPosition,
                                        desiredOrientation,
                                        desiredLinearVelocity,
                                        desiredAngularVelocity,
                                        MessageTools.toFrameId(trajectoryReferenceFrame));
   }

   public static void configureForStreaming(WholeBodyTrajectoryMessage messageToModify, double streamIntegrationDuration, long timestamp)
   {
      configureForStreaming(messageToModify.getHeadTrajectoryMessage().getSo3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getNeckTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getChestTrajectoryMessage().getSo3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getPelvisTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getLeftArmTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getRightArmTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getLeftFootTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getRightFootTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getLeftHandTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getRightHandTrajectoryMessage().getSe3Trajectory(), streamIntegrationDuration, timestamp);
      configureForStreaming(messageToModify.getSpineTrajectoryMessage().getJointspaceTrajectory(), streamIntegrationDuration, timestamp);
   }

   public static void configureForStreaming(JointspaceTrajectoryMessage messageToModify, double streamIntegrationDuration, long timestamp)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration, timestamp);
   }

   public static void configureForStreaming(EuclideanTrajectoryMessage messageToModify, double streamIntegrationDuration, long timestamp)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration, timestamp);
   }

   public static void configureForStreaming(SO3TrajectoryMessage messageToModify, double streamIntegrationDuration, long timestamp)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration, timestamp);
   }

   public static void configureForStreaming(SE3TrajectoryMessage messageToModify, double streamIntegrationDuration, long timestamp)
   {
      configureForStreaming(messageToModify.getQueueingProperties(), streamIntegrationDuration, timestamp);
   }

   public static void configureForStreaming(QueueableMessage messageToModify, double streamIntegrationDuration, long timestamp)
   {
      messageToModify.setExecutionMode(ExecutionMode.STREAM.toByte());
      messageToModify.setStreamIntegrationDuration(streamIntegrationDuration);
      messageToModify.setTimestamp(timestamp);
   }

   public static void configureForOverriding(WholeBodyTrajectoryMessage messageToModify)
   {
      configureForOverriding(messageToModify.getHeadTrajectoryMessage().getSo3Trajectory());
      configureForOverriding(messageToModify.getChestTrajectoryMessage().getSo3Trajectory());
      configureForOverriding(messageToModify.getPelvisTrajectoryMessage().getSe3Trajectory());
      configureForOverriding(messageToModify.getLeftArmTrajectoryMessage().getJointspaceTrajectory());
      configureForOverriding(messageToModify.getRightArmTrajectoryMessage().getJointspaceTrajectory());
      configureForOverriding(messageToModify.getLeftFootTrajectoryMessage().getSe3Trajectory());
      configureForOverriding(messageToModify.getRightFootTrajectoryMessage().getSe3Trajectory());
      configureForOverriding(messageToModify.getLeftHandTrajectoryMessage().getSe3Trajectory());
      configureForOverriding(messageToModify.getRightHandTrajectoryMessage().getSe3Trajectory());
      configureForOverriding(messageToModify.getSpineTrajectoryMessage().getJointspaceTrajectory());
   }

   public static void configureForOverriding(JointspaceTrajectoryMessage messageToModify)
   {
      configureForOverriding(messageToModify.getQueueingProperties());
   }

   public static void configureForOverriding(EuclideanTrajectoryMessage messageToModify)
   {
      configureForOverriding(messageToModify.getQueueingProperties());
   }

   public static void configureForOverriding(SO3TrajectoryMessage messageToModify)
   {
      configureForOverriding(messageToModify.getQueueingProperties());
   }

   public static void configureForOverriding(SE3TrajectoryMessage messageToModify)
   {
      configureForOverriding(messageToModify.getQueueingProperties());
   }

   public static void configureForOverriding(QueueableMessage messageToModify)
   {
      messageToModify.setExecutionMode(ExecutionMode.OVERRIDE.toByte());
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

   public static SE3TrajectoryPointMessage createSE3TrajectoryPointMessage(double time,
                                                                           Point3DReadOnly position,
                                                                           Orientation3DReadOnly orientation,
                                                                           Vector3DReadOnly linearVelocity,
                                                                           Vector3DReadOnly angularVelocity)
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

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3DReadOnly location, Orientation3DReadOnly orientation)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide,
                                                               Point3DReadOnly location,
                                                               Orientation3DReadOnly orientation,
                                                               List<? extends Point2DReadOnly> predictedContactPoints)
   {
      return createFootstepDataMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide,
                                                               Point3DReadOnly location,
                                                               Orientation3DReadOnly orientation,
                                                               TrajectoryType trajectoryType,
                                                               double swingHeight)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null, trajectoryType, swingHeight);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide,
                                                               Point3DReadOnly location,
                                                               Orientation3DReadOnly orientation,
                                                               List<? extends Point2DReadOnly> predictedContactPoints,
                                                               TrajectoryType trajectoryType,
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

      if (!footstep.getCustomPositionWaypoints().isEmpty())
      {
         if (footstep.getCustomPositionWaypoints().size() != 2)
         {
            LogTools.warn("Received footstep object without the correct number of waypoint positions. Should be 0 or 2, received: "
                          + footstep.getCustomPositionWaypoints().size());
         }
         else
         {
            for (int i = 0; i < 2; i++)
            {
               FramePoint3D framePoint = footstep.getCustomPositionWaypoints().get(i);
               framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
               message.getCustomPositionWaypoints().add().set(framePoint);
            }
         }
      }

      if (!footstep.getCustomWaypointProportions().isEmpty())
      {
         if (footstep.getCustomWaypointProportions().size() != 2)
         {
            LogTools.warn("Received footstep object without the correct number of waypoint proportions. Should be 0 or 2, received: "
                          + footstep.getCustomWaypointProportions().size());
         }
         else
         {
            message.getCustomWaypointProportions().clear();
            for (int i = 0; i < 2; i++)
            {
               message.getCustomWaypointProportions().add(footstep.getCustomWaypointProportions().get(i).getValue());
            }
         }
      }

      for (int i = 0; i < footstep.getSwingTrajectory().size(); i++)
      {
         FrameSE3TrajectoryPoint swingTrajectoryPoint = footstep.getSwingTrajectory().get(i);
         SE3TrajectoryPointMessage swingTrajectoryPointToSet = message.getSwingTrajectory().add();

         swingTrajectoryPointToSet.getPosition().set(swingTrajectoryPoint.getPosition());
         swingTrajectoryPointToSet.getOrientation().set(swingTrajectoryPoint.getOrientation());
         swingTrajectoryPointToSet.getLinearVelocity().set(swingTrajectoryPoint.getLinearVelocity());
         swingTrajectoryPointToSet.getAngularVelocity().set(swingTrajectoryPoint.getAngularVelocity());
         swingTrajectoryPointToSet.setTime(swingTrajectoryPoint.getTime());
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
      message.getControlFramePositionInEndEffector().set(transformToBodyFixedFrame.getTranslation());
      message.getControlFrameOrientationInEndEffector().set(transformToBodyFixedFrame.getRotation());

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

   public static CenterOfMassTrajectoryMessage createCenterOfMassTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition)
   {
      CenterOfMassTrajectoryMessage message = new CenterOfMassTrajectoryMessage();
      message.getEuclideanTrajectory().set(createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, ReferenceFrame.getWorldFrame()));
      return message;
   }

   /**
    * Use this constructor to execute a straight line trajectory for center of mass with velocity.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime         how long it takes to reach the desired position.
    * @param desiredPosition        desired center of mass position expressed in world frame.
    * @param desiredLinearVelocity  desired linear velocity at the end of the trajectory.
    */
   public static CenterOfMassTrajectoryMessage createCenterOfMassTrajectoryMessage(double trajectoryTime,
                                                                                   Point3DReadOnly desiredPosition,
                                                                                   Vector3DReadOnly desiredLinearVelocity)
   {
      CenterOfMassTrajectoryMessage message = new CenterOfMassTrajectoryMessage();
      message.getEuclideanTrajectory().set(createEuclideanTrajectoryMessage(trajectoryTime, desiredPosition, desiredLinearVelocity, ReferenceFrame.getWorldFrame()));
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

   public static LegTrajectoryMessage createLegTrajectoryMessage(RobotSide robotSide, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      LegTrajectoryMessage message = new LegTrajectoryMessage();
      message.getJointspaceTrajectory().set(new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Use this constructor to go straight to the given end points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide             is used to define which leg is performing the trajectory.
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of leg joints.
    */
   public static LegTrajectoryMessage createLegTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      return createLegTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions, null, null);
   }

   /**
    * Use this constructor to go straight to the given end points using the specified qp weights. Set
    * the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide             is used to define which leg is performing the trajectory.
    * @param trajectoryTime        how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the
    *                              number of leg joints.
    * @param weights               the qp weights for the joint accelerations. If any index is set to
    *                              NaN, that joint will use the controller default weight
    */
   public static LegTrajectoryMessage createLegTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      return createLegTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions, null, weights);
   }

   /**
    * Use this constructor to go straight to the given end points with final velocity using the
    * specified qp weights. Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide              is used to define which leg is performing the trajectory.
    * @param trajectoryTime         how long it takes to reach the desired pose.
    * @param desiredJointPositions  desired joint positions. The array length should be equal to the
    *                               number of leg joints.
    * @param desiredJointVelocities desired final joint velocities. The array length should be equal to
    *                               the number of leg joints. Can be {@code null}.
    * @param weights                the qp weights for the joint accelerations. If any index is set to
    *                               NaN, that joint will use the controller default weight. Can be
    *                               {@code null}.
    */
   public static LegTrajectoryMessage createLegTrajectoryMessage(RobotSide robotSide,
                                                                 double trajectoryTime,
                                                                 double[] desiredJointPositions,
                                                                 double[] desiredJointVelocities,
                                                                 double[] weights)
   {
      LegTrajectoryMessage message = new LegTrajectoryMessage();
      message.getJointspaceTrajectory().set(createJointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, desiredJointVelocities, weights));
      message.setRobotSide(robotSide.toByte());
      return message;
   }

   /**
    * Create a message using the given joint trajectory points. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param robotSide                     is used to define which leg is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public static LegTrajectoryMessage createLegTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      LegTrajectoryMessage message = new LegTrajectoryMessage();
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
    * @param robotSide is used to define which leg is performing the trajectory.
    */
   public static LegTrajectoryMessage createLegTrajectoryMessage(RobotSide robotSide)
   {
      LegTrajectoryMessage message = new LegTrajectoryMessage();
      message.setRobotSide(robotSide.toByte());
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
   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Point3DReadOnly desiredPosition,
                                                                   Orientation3DReadOnly desiredOrientation)
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

   public static FootTrajectoryMessage createFootTrajectoryMessage(RobotSide robotSide,
                                                                   double trajectoryTime,
                                                                   Pose3DReadOnly desiredPose,
                                                                   SpatialVectorReadOnly desiredVelocity,
                                                                   ReferenceFrame trajectoryReferenceFrame)
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

   public static void checkRobotSide(HumanoidBodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
   }

   @Deprecated
   public static IntrinsicParametersMessage toIntrinsicParametersMessage(Object intrinsicParameters)
   {
      IntrinsicParametersMessage intrinsicParametersMessage = new IntrinsicParametersMessage();
      //      intrinsicParametersMessage.setWidth(intrinsicParameters.width);
      //      intrinsicParametersMessage.setHeight(intrinsicParameters.height);
      //      intrinsicParametersMessage.setFx(intrinsicParameters.fx);
      //      intrinsicParametersMessage.setFy(intrinsicParameters.fy);
      //      intrinsicParametersMessage.setSkew(intrinsicParameters.skew);
      //      intrinsicParametersMessage.setCx(intrinsicParameters.cx);
      //      intrinsicParametersMessage.setCy(intrinsicParameters.cy);
      //      if (intrinsicParameters.radial != null)
      //         intrinsicParametersMessage.getRadial().add(intrinsicParameters.radial);
      //      intrinsicParametersMessage.setT1(intrinsicParameters.t1);
      //      intrinsicParametersMessage.setT2(intrinsicParameters.t2);
      return intrinsicParametersMessage;
   }

   @Deprecated
   public static Object toIntrinsicParameters(IntrinsicParametersMessage message)
   {
      //      CameraPinholeBrown intrinsicParameters = new CameraPinholeBrown();
      //      intrinsicParameters.width = message.getWidth();
      //      intrinsicParameters.height = message.getHeight();
      //      intrinsicParameters.fx = message.getFx();
      //      intrinsicParameters.fy = message.getFy();
      //      intrinsicParameters.skew = message.getSkew();
      //      intrinsicParameters.cx = message.getCx();
      //      intrinsicParameters.cy = message.getCy();
      //      if (!message.getRadial().isEmpty())
      //         intrinsicParameters.radial = message.getRadial().toArray();
      //      intrinsicParameters.t1 = message.getT1();
      //      intrinsicParameters.t2 = message.getT2();
      return new Object();
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


   public static void checkIfDataFrameIdsMatch(FrameInformation frameInformation, ReferenceFrame referenceFrame)
   {
      long expectedId = HumanoidMessageTools.getDataFrameIDConsideringDefault(frameInformation);

      if (expectedId != referenceFrame.getFrameNameHashCode() && expectedId != referenceFrame.getAdditionalNameBasedHashCode())
      {
         String msg = "Argument's hashcode " + referenceFrame + " " + referenceFrame.getFrameNameHashCode() + " does not match " + expectedId;
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
      if (dataId == EuclidHashCodeTools.DEFAULT_HASHCODE)
      {
         dataId = frameInformation.getTrajectoryReferenceFrameId();
      }
      return dataId;
   }

   public static double unpackJointAngle(HandJointAnglePacket handJointAnglePacket, HandJointName jointName)
   {
      int index = jointName.getIndex(RobotSide.fromByte(handJointAnglePacket.getRobotSide()));
      if (index == -1 || index >= handJointAnglePacket.getJointAngles().size())
      {
         return 0;
      }

      return handJointAnglePacket.getJointAngles().get(index);
   }

   public static void packFootSupportPolygon(RobotSide robotSide, FrameConvexPolygon2DReadOnly footPolygon, CapturabilityBasedStatus capturabilityBasedStatus)
   {
      int numberOfVertices = footPolygon.getNumberOfVertices();

      if (numberOfVertices > CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES)
      {
         numberOfVertices = CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES;
      }

      if (robotSide == RobotSide.LEFT)
      {
         capturabilityBasedStatus.getLeftFootSupportPolygon3d().clear();
      }
      else
      {
         capturabilityBasedStatus.getRightFootSupportPolygon3d().clear();
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point3D vertex3D;

         if (robotSide == RobotSide.LEFT)
            vertex3D = capturabilityBasedStatus.getLeftFootSupportPolygon3d().add();
         else
            vertex3D = capturabilityBasedStatus.getRightFootSupportPolygon3d().add();

         vertex3D.set(footPolygon.getVertex(i), 0.0);
         footPolygon.getReferenceFrame().transformFromThisToDesiredFrame(ReferenceFrame.getWorldFrame(), vertex3D);

      }
   }

   public static FrameConvexPolygon2D unpackFootSupportPolygon(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT && !capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty())
         return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame(),
                                         Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getLeftFootSupportPolygon3d()));
      else if (capturabilityBasedStatus.getRightFootSupportPolygon3d() != null)
         return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame(),
                                         Vertex3DSupplier.asVertex3DSupplier(capturabilityBasedStatus.getRightFootSupportPolygon3d()));
      else
         return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame());
   }

   public static boolean unpackIsInDoubleSupport(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      return !capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty() & !capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty();
   }

   public static boolean unpackIsSupportFoot(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotside)
   {
      if (robotside == RobotSide.LEFT)
         return !capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty();
      else
         return !capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty();
   }

   public static boolean isHandLoadBearing(RobotSide robotSide, CapturabilityBasedStatus capturabilityBasedStatus)
   {
      if (robotSide == RobotSide.LEFT)
         return !capturabilityBasedStatus.getLeftHandContactPoints().isEmpty();
      else
         return !capturabilityBasedStatus.getRightHandContactPoints().isEmpty();
   }

   public static boolean unpackIsSupportHand(CapturabilityBasedStatus capturabilityBasedStatus, RobotSide robotSide, FullHumanoidRobotModel fullRobotModel, FramePoint3DBasics contactPointToPack)
   {
      List<Point3D> handContactPointList = robotSide == RobotSide.LEFT ? capturabilityBasedStatus.getLeftHandContactPoints() : capturabilityBasedStatus.getRightHandContactPoints();
      boolean isLoadBearing = !handContactPointList.isEmpty();

      if (isLoadBearing)
         contactPointToPack.setIncludingFrame(fullRobotModel.getHand(robotSide).getBodyFixedFrame(), handContactPointList.get(0));
      else
         contactPointToPack.setToNaN();

      return isLoadBearing;
   }

   public static void packManifold(byte[] configurationSpaces, double[] lowerLimits, double[] upperLimits, ReachingManifoldMessage reachingManifoldMessage)
   {
      if (configurationSpaces.length != lowerLimits.length || configurationSpaces.length != upperLimits.length || lowerLimits.length != upperLimits.length)
         throw new RuntimeException("Inconsistent array lengths: configurationSpaces = " + configurationSpaces.length);

      reachingManifoldMessage.getManifoldConfigurationSpaceNames().resetQuick();
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

   /**
    * Empty message used to reset any {@link WholeBodyTrajectoryMessage}. Do NOT modify.
    */
   private static final WholeBodyTrajectoryMessage EMPTY_WHOLE_BODY_TRAJECTORY_MESSAGE = new WholeBodyTrajectoryMessage();

   public static void resetWholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryMessage message)
   {
      message.set(EMPTY_WHOLE_BODY_TRAJECTORY_MESSAGE);
   }

   private static final WholeBodyStreamingMessage EMPTY_WHOLE_BODY_STREAMING_MESSAGE = new WholeBodyStreamingMessage();

   public static void resetWholeBodyStreamingMessage(WholeBodyStreamingMessage message)
   {
      message.set(EMPTY_WHOLE_BODY_STREAMING_MESSAGE);
   }
}
