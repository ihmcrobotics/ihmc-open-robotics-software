package us.ihmc.humanoidRobotics.communication.packets;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.*;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.BiConsumer;

public class KinematicsToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final OneDoFJointBasics[] neckJoints;
   private final SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();

   public KinematicsToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();

      if (head == null)
         neckJoints = new OneDoFJointBasics[0];
      else
         neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         armJoints.put(robotSide, MultiBodySystemTools.createOneDoFJointPath(chest, hand));
      }
   }

   public void updateFullRobotModel(BiConsumer<FloatingJointBasics, OneDoFJointBasics[]> robotUpdater)
   {
      robotUpdater.accept(rootJoint, oneDoFJoints);
      referenceFrames.updateFrames();
   }

   public void updateFullRobotModel(KinematicsToolboxOutputStatus solution)
   {
      MessageTools.unpackDesiredJointState(solution, rootJoint, oneDoFJoints);
      referenceFrames.updateFrames();
   }

   private WholeBodyTrajectoryMessage output;

   public void setMessageToCreate(WholeBodyTrajectoryMessage message)
   {
      output = message;
   }

   private double trajectoryTime = Double.NaN;

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   private boolean enableVelocity = false;

   public void setEnableVelocity(boolean enable)
   {
      enableVelocity = enable;
   }

   public void computeArmTrajectoryMessages()
   {
      checkIfDataHasBeenSet();

      for (RobotSide robotSide : RobotSide.values)
         computeArmTrajectoryMessage(robotSide);
   }

   public void computeArmTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      OneDoFJointBasics[] joints = armJoints.get(robotSide);
      int numberOfArmJoints = joints.length;
      ArmTrajectoryMessage armTrajectoryMessage = select(robotSide, output.getLeftArmTrajectoryMessage(), output.getRightArmTrajectoryMessage());
      armTrajectoryMessage.setRobotSide(robotSide.toByte());
      JointspaceTrajectoryMessage jointspaceTrajectory = armTrajectoryMessage.getJointspaceTrajectory();
      jointspaceTrajectory.getJointTrajectoryMessages().clear();

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         OneDoFJointBasics joint = joints[i];
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = jointspaceTrajectory.getJointTrajectoryMessages().add();
         jointTrajectoryMessage.getTrajectoryPoints().clear();
         packTrajectoryPoint1DMessage(trajectoryTime,
                                      getJointPosition(joint),
                                      enableVelocity ? joint.getQd() : 0.0,
                                      jointTrajectoryMessage.getTrajectoryPoints().add());
      }
   }

   public void computeHandTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandTrajectoryMessage(robotSide);
   }

   private final FramePose3D desiredPose = new FramePose3D(worldFrame);
   private final SpatialVector desiredSpatialVelocity = new SpatialVector();

   public void computeHandTrajectoryMessage(RobotSide robotSide)
   {
      computeHandTrajectoryMessage(robotSide, worldFrame);
   }

   public void computeHandTrajectoryMessage(RobotSide robotSide, ReferenceFrame trajectoryFrame)
   {
      computeHandTrajectoryMessage(robotSide, trajectoryFrame.getFrameNameHashCode());
   }

   public void computeHandTrajectoryMessage(RobotSide robotSide, long trajectoryFrameId)
   {
      checkIfDataHasBeenSet();

      // TODO Add the option to define the control frame in the API instead of hardcoding it here.
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      desiredPose.setToZero(handControlFrame);
      desiredPose.changeFrame(worldFrame);
      spatialVelocity(handControlFrame, worldFrame, enableVelocity, desiredSpatialVelocity);
      HandTrajectoryMessage handTrajectoryMessage = select(robotSide, output.getLeftHandTrajectoryMessage(), output.getRightHandTrajectoryMessage());
      handTrajectoryMessage.setRobotSide(robotSide.toByte());
      SE3TrajectoryMessage se3TrajectoryMessage = handTrajectoryMessage.getSe3Trajectory();
      packCustomControlFrame(fullRobotModel.getHand(robotSide).getBodyFixedFrame(), handControlFrame, se3TrajectoryMessage);
      se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrameId);
      se3TrajectoryMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.getFrameNameHashCode());

      Object<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = se3TrajectoryMessage.getTaskspaceTrajectoryPoints();
      taskspaceTrajectoryPoints.clear();
      packSE3TrajectoryPointMessage(trajectoryTime, desiredPose, desiredSpatialVelocity, taskspaceTrajectoryPoints.add());
   }

   public void computeNeckTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      int numberOfJoints = neckJoints.length;

      JointspaceTrajectoryMessage jointspaceTrajectory = output.getNeckTrajectoryMessage().getJointspaceTrajectory();
      jointspaceTrajectory.getJointTrajectoryMessages().clear();

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJointBasics joint = neckJoints[i];
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = jointspaceTrajectory.getJointTrajectoryMessages().add();
         jointTrajectoryMessage.getTrajectoryPoints().clear();
         packTrajectoryPoint1DMessage(trajectoryTime,
                                      getJointPosition(joint),
                                      enableVelocity ? joint.getQd() : 0.0,
                                      jointTrajectoryMessage.getTrajectoryPoints().add());
      }
   }

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();

   public void computeHeadTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame headFrame = fullRobotModel.getHead().getBodyFixedFrame();
      desiredOrientation.setToZero(headFrame);
      desiredOrientation.changeFrame(worldFrame);
      angularVelocity(headFrame, worldFrame, enableVelocity, desiredAngularVelocity);

      HeadTrajectoryMessage headTrajectoryMessage = output.getHeadTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = headTrajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(worldFrame.getFrameNameHashCode());

      Object<SO3TrajectoryPointMessage> taskspaceTrajectoryPoints = so3Trajectory.getTaskspaceTrajectoryPoints();
      taskspaceTrajectoryPoints.clear();
      packSO3TrajectoryPointMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, taskspaceTrajectoryPoints.add());
   }

   public void computeChestTrajectoryMessage()
   {
      computeChestTrajectoryMessage(referenceFrames.getPelvisZUpFrame());
   }

   public void computeChestTrajectoryMessage(ReferenceFrame trajectoryFrame)
   {
      computeChestTrajectoryMessage(trajectoryFrame.getFrameNameHashCode());
   }

   public void computeChestTrajectoryMessage(long trajectoryFrameId)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      desiredOrientation.setToZero(chestFrame);
      desiredOrientation.changeFrame(worldFrame);
      angularVelocity(chestFrame, worldFrame, enableVelocity, desiredAngularVelocity);

      ChestTrajectoryMessage chestTrajectoryMessage = output.getChestTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrameId);
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(worldFrame.getFrameNameHashCode());

      Object<SO3TrajectoryPointMessage> taskspaceTrajectoryPoints = so3Trajectory.getTaskspaceTrajectoryPoints();
      taskspaceTrajectoryPoints.clear();
      packSO3TrajectoryPointMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, taskspaceTrajectoryPoints.add());
   }

   public void computePelvisTrajectoryMessage()
   {
      computePelvisTrajectoryMessage(worldFrame);
   }

   public void computePelvisTrajectoryMessage(ReferenceFrame trajectoryFrame)
   {
      computePelvisTrajectoryMessage(trajectoryFrame.getFrameNameHashCode());
   }

   public void computePelvisTrajectoryMessage(long trajectoryFrameId)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame pelvisFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      desiredPose.setToZero(pelvisFrame);
      desiredPose.changeFrame(worldFrame);
      spatialVelocity(pelvisFrame, worldFrame, enableVelocity, desiredSpatialVelocity);

      PelvisTrajectoryMessage pelvisTrajectoryMessage = output.getPelvisTrajectoryMessage();
      SE3TrajectoryMessage se3Trajectory = pelvisTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrameId);
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(worldFrame.getFrameNameHashCode());

      Object<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = se3Trajectory.getTaskspaceTrajectoryPoints();
      taskspaceTrajectoryPoints.clear();
      packSE3TrajectoryPointMessage(trajectoryTime, desiredPose, desiredSpatialVelocity, taskspaceTrajectoryPoints.add());
   }

   public void computeFootTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeFootTrajectoryMessage(robotSide);
   }

   public void computeFootTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame footFrame = fullRobotModel.getEndEffector(robotSide, LimbName.LEG).getBodyFixedFrame();
      desiredPose.setToZero(footFrame);
      desiredPose.changeFrame(worldFrame);
      spatialVelocity(footFrame, worldFrame, enableVelocity, desiredSpatialVelocity);

      FootTrajectoryMessage footTrajectoryMessage = select(robotSide, output.getLeftFootTrajectoryMessage(), output.getRightFootTrajectoryMessage());
      SE3TrajectoryMessage se3Trajectory = footTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(worldFrame.getFrameNameHashCode());

      Object<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = se3Trajectory.getTaskspaceTrajectoryPoints();
      taskspaceTrajectoryPoints.clear();
      packSE3TrajectoryPointMessage(trajectoryTime, desiredPose, desiredSpatialVelocity, taskspaceTrajectoryPoints.add());
   }

   private static <T> T select(RobotSide robotSide, T left, T right)
   {
      return robotSide == RobotSide.LEFT ? left : right;
   }

   private static void angularVelocity(MovingReferenceFrame movingFrame,
                                       ReferenceFrame outputFrame,
                                       boolean enableVelocity,
                                       FrameVector3DBasics angularVelocityToPack)
   {
      if (!enableVelocity)
      {
         angularVelocityToPack.setToZero(outputFrame);
      }
      else
      {
         angularVelocityToPack.setIncludingFrame(movingFrame.getTwistOfFrame().getAngularPart());
         angularVelocityToPack.changeFrame(outputFrame);
      }
   }

   private static void spatialVelocity(MovingReferenceFrame movingFrame,
                                       ReferenceFrame outputFrame,
                                       boolean enableVelocity,
                                       SpatialVectorBasics spatialVelocityToPack)
   {
      if (!enableVelocity)
      {
         spatialVelocityToPack.setToZero(outputFrame);
      }
      else
      {
         spatialVelocityToPack.setIncludingFrame(movingFrame.getTwistOfFrame());
         spatialVelocityToPack.changeFrame(outputFrame);
      }
   }

   public static double getJointPosition(OneDoFJointReadOnly joint)
   {
      return MathTools.clamp(joint.getQ(), joint.getJointLimitLower(), joint.getJointLimitUpper());
   }

   public static void packTrajectoryPoint1DMessage(double time, double position, double velocity, TrajectoryPoint1DMessage messageToPack)
   {
      messageToPack.setTime(time);
      messageToPack.setPosition(position);
      messageToPack.setVelocity(velocity);
   }

   public static void packCustomControlFrame(ReferenceFrame endEffectorFrame, ReferenceFrame controlFrame, SE3TrajectoryMessage messageToPack)
   {
      messageToPack.setUseCustomControlFrame(true);
      Pose3D controlFramePose = messageToPack.getControlFramePose();
      controlFramePose.setToZero();
      controlFrame.transformFromThisToDesiredFrame(endEffectorFrame, controlFramePose);
   }

   public static void packSO3TrajectoryPointMessage(double time,
                                                    Orientation3DReadOnly orientation,
                                                    Vector3DReadOnly angularVelocity,
                                                    SO3TrajectoryPointMessage messageToPack)
   {
      messageToPack.setTime(time);
      messageToPack.getOrientation().set(orientation);
      messageToPack.getAngularVelocity().set(angularVelocity);
   }

   public static void packSE3TrajectoryPointMessage(double time,
                                                    Pose3DReadOnly pose,
                                                    SpatialVectorReadOnly spatialVelocity,
                                                    SE3TrajectoryPointMessage messageToPack)
   {
      messageToPack.setTime(time);
      messageToPack.getPosition().set(pose.getPosition());
      messageToPack.getOrientation().set(pose.getOrientation());
      messageToPack.getLinearVelocity().set(spatialVelocity.getLinearPart());
      messageToPack.getAngularVelocity().set(spatialVelocity.getAngularPart());
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   private void checkIfDataHasBeenSet()
   {
      if (output == null)
         throw new RuntimeException("Need to call setMessageToCreate() first.");
      if (Double.isNaN(trajectoryTime))
         throw new RuntimeException("Need to call setTrajectoryTime() first.");
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }
}
