package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.JointspaceStreamingMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.SE3StreamingMessage;
import controller_msgs.msg.dds.SO3StreamingMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KSTStreamingMessageFactory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final OneDoFJointBasics[] neckJoints;
   private final SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();

   public KSTStreamingMessageFactory(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();

      neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         armJoints.put(robotSide, MultiBodySystemTools.createOneDoFJointPath(chest, hand));
      }
   }

   public void updateFullRobotModel(KinematicsToolboxOutputStatus solution)
   {
      MessageTools.unpackDesiredJointState(solution, rootJoint, oneDoFJoints);
      referenceFrames.updateFrames();
   }

   private WholeBodyStreamingMessage output;

   public void setMessageToCreate(WholeBodyStreamingMessage message)
   {
      output = message;
   }

   private boolean enableVelocity = false;

   public void setEnableVelocity(boolean enable)
   {
      enableVelocity = enable;
   }

   public void computeArmStreamingMessages()
   {
      checkIfDataHasBeenSet();

      for (RobotSide robotSide : RobotSide.values)
         computeArmStreamingMessage(robotSide);
   }

   public void computeArmStreamingMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      OneDoFJointBasics[] joints = armJoints.get(robotSide);
      int numberOfArmJoints = joints.length;
      JointspaceStreamingMessage armStreamingMessage = select(robotSide, output.getLeftArmStreamingMessage(), output.getRightArmStreamingMessage());

      armStreamingMessage.getPositions().reset();
      armStreamingMessage.getVelocities().reset();

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         OneDoFJointBasics joint = joints[i];
         armStreamingMessage.getPositions().add(getJointPosition(joint));
         armStreamingMessage.getVelocities().add((float) (enableVelocity ? joint.getQd() : 0.0));
      }

      if (robotSide == RobotSide.LEFT)
         output.setHasLeftArmStreamingMessage(true);
      else
         output.setHasRightArmStreamingMessage(true);
   }

   public void computeHandStreamingMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandStreamingMessage(robotSide);
   }

   private final FramePose3D desiredPose = new FramePose3D(worldFrame);
   private final SpatialVector desiredSpatialVelocity = new SpatialVector();

   public void computeHandStreamingMessage(RobotSide robotSide)
   {
      computeHandStreamingMessage(robotSide, worldFrame);
   }

   public void computeHandStreamingMessage(RobotSide robotSide, ReferenceFrame trajectoryFrame)
   {
      checkIfDataHasBeenSet();

      // TODO Add the option to define the control frame in the API instead of hardcoding it here.
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      desiredPose.setToZero(handControlFrame);
      desiredPose.changeFrame(worldFrame);
      spatialVelocity(handControlFrame, worldFrame, enableVelocity, desiredSpatialVelocity);
      SE3StreamingMessage handStreamingMessage = select(robotSide, output.getLeftHandStreamingMessage(), output.getRightHandStreamingMessage());
      packCustomControlFrame(fullRobotModel.getHand(robotSide).getBodyFixedFrame(), handControlFrame, handStreamingMessage);
      handStreamingMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrame.hashCode());
      handStreamingMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.hashCode());

      packSE3TrajectoryPointMessage(desiredPose, desiredSpatialVelocity, handStreamingMessage);

      if (robotSide == RobotSide.LEFT)
         output.setHasLeftHandStreamingMessage(true);
      else
         output.setHasRightHandStreamingMessage(true);
   }

   public void computeNeckStreamingMessage()
   {
      checkIfDataHasBeenSet();

      int numberOfJoints = neckJoints.length;

      JointspaceStreamingMessage neckStreamingMessage = output.getNeckStreamingMessage();

      neckStreamingMessage.getPositions().reset();
      neckStreamingMessage.getVelocities().reset();

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJointBasics joint = neckJoints[i];
         neckStreamingMessage.getPositions().add((float) getJointPosition(joint));
         neckStreamingMessage.getVelocities().add((float) (enableVelocity ? joint.getQd() : 0.0));
      }

      output.setHasNeckStreamingMessage(true);
   }

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();

   public void computeChestStreamingMessage()
   {
      computeChestStreamingMessage(referenceFrames.getPelvisZUpFrame());
   }

   public void computeChestStreamingMessage(ReferenceFrame trajectoryFrame)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      desiredOrientation.setToZero(chestFrame);
      desiredOrientation.changeFrame(worldFrame);
      angularVelocity(chestFrame, worldFrame, enableVelocity, desiredAngularVelocity);

      SO3StreamingMessage chestStreamingMessage = output.getChestStreamingMessage();
      chestStreamingMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrame.hashCode());
      chestStreamingMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.hashCode());

      packSO3TrajectoryPointMessage(desiredOrientation, desiredAngularVelocity, chestStreamingMessage);

      output.setHasChestStreamingMessage(true);
   }

   public void computePelvisStreamingMessage()
   {
      computePelvisStreamingMessage(worldFrame);
   }

   public void computePelvisStreamingMessage(ReferenceFrame trajectoryFrame)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame pelvisFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      desiredPose.setToZero(pelvisFrame);
      desiredPose.changeFrame(worldFrame);
      spatialVelocity(pelvisFrame, worldFrame, enableVelocity, desiredSpatialVelocity);

      SE3StreamingMessage pelvisStreamingMessage = output.getPelvisStreamingMessage();
      pelvisStreamingMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrame.hashCode());
      pelvisStreamingMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.hashCode());

      packSE3TrajectoryPointMessage(desiredPose, desiredSpatialVelocity, pelvisStreamingMessage);

      output.setHasPelvisStreamingMessage(true);
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

   public static float getJointPosition(OneDoFJointReadOnly joint)
   {
      return (float) MathTools.clamp((float) joint.getQ(), joint.getJointLimitLower() + 1.0e-7, joint.getJointLimitUpper() - 1.0e-7);
   }

   public static void packCustomControlFrame(ReferenceFrame endEffectorFrame, ReferenceFrame controlFrame, SE3StreamingMessage messageToPack)
   {
      messageToPack.setUseCustomControlFrame(true);
      Pose3D controlFramePose = messageToPack.getControlFramePose();
      controlFramePose.setToZero();
      controlFrame.transformFromThisToDesiredFrame(endEffectorFrame, controlFramePose);
   }

   public static void packSO3TrajectoryPointMessage(Orientation3DReadOnly orientation, Vector3DReadOnly angularVelocity, SO3StreamingMessage messageToPack)
   {
      messageToPack.getOrientation().set(orientation);
      messageToPack.getAngularVelocity().set(angularVelocity);
   }

   public static void packSE3TrajectoryPointMessage(Pose3DReadOnly pose, SpatialVectorReadOnly spatialVelocity, SE3StreamingMessage messageToPack)
   {
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
