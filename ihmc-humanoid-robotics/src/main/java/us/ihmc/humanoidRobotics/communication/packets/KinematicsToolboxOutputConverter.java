package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Arrays;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;

public class KinematicsToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModelToUseForConversion;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final int jointsHashCode;

   public KinematicsToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModelToUseForConversion = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModelToUseForConversion.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUseForConversion);
      jointsHashCode = Arrays.hashCode(oneDoFJoints);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModelToUseForConversion);
   }

   public void updateFullRobotModel(KinematicsToolboxOutputStatus solution)
   {
      if (jointsHashCode != solution.getJointNameHash())
         throw new RuntimeException("Hashes are different.");

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         float q = solution.getDesiredJointAngles().get(i);
         OneDoFJointBasics joint = oneDoFJoints[i];
         joint.setQ(q);
         if (solution.getDesiredJointAngles().size() == solution.getDesiredJointVelocities().size())
         {
            float qd = solution.getDesiredJointVelocities().get(i);
            joint.setQd(qd);
         }
      }
      Vector3D translation = solution.getDesiredRootTranslation();
      rootJoint.getJointPose().setPosition(translation.getX(), translation.getY(), translation.getZ());
      Quaternion orientation = solution.getDesiredRootOrientation();
      rootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
      fullRobotModelToUseForConversion.updateFrames();
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

   public void computeArmTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeArmTrajectoryMessage(robotSide);
   }

   public void computeArmTrajectoryMessage(RobotSide robotSide)
   {
      RigidBodyBasics hand = fullRobotModelToUseForConversion.getHand(robotSide);
      RigidBodyBasics chest = fullRobotModelToUseForConversion.getChest();
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
      int numberOfArmJoints = armJoints.length;
      double[] desiredJointPositions = new double[numberOfArmJoints];
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         OneDoFJointBasics armJoint = armJoints[i];
         desiredJointPositions[i] = MathTools.clamp(armJoint.getQ(), armJoint.getJointLimitLower(), armJoint.getJointLimitUpper());
      }
      ArmTrajectoryMessage armTrajectoryMessage = robotSide == RobotSide.LEFT ? output.getLeftArmTrajectoryMessage() : output.getRightArmTrajectoryMessage();
      armTrajectoryMessage.set(HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions));
   }

   public void computeHandTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandTrajectoryMessage(robotSide);
   }

   public void computeHandTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      ReferenceFrame trajectoryFrame = worldFrame;
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame handControlFrame = fullRobotModelToUseForConversion.getHandControlFrame(robotSide);
      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.changeFrame(worldFrame);
      desiredHandPose.get(desiredPosition, desiredOrientation);
      HandTrajectoryMessage handTrajectoryMessage = robotSide == RobotSide.LEFT ? output.getLeftHandTrajectoryMessage()
            : output.getRightHandTrajectoryMessage();
      handTrajectoryMessage.set(HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation,
                                                                                 trajectoryFrame));
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
   }

   public void computeChestTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      ReferenceFrame chestFrame = fullRobotModelToUseForConversion.getChest().getBodyFixedFrame();
      Quaternion desiredQuaternion = new Quaternion();
      FrameQuaternion desiredOrientation = new FrameQuaternion(chestFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredQuaternion.set(desiredOrientation);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredQuaternion, worldFrame,
                                                                                                        pelvisZUpFrame);
      output.getChestTrajectoryMessage().set(chestTrajectoryMessage);
   }

   public void computePelvisTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame pelvisFrame = fullRobotModelToUseForConversion.getRootJoint().getFrameAfterJoint();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvisFrame);
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.get(desiredPosition, desiredOrientation);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);
      output.getPelvisTrajectoryMessage().set(pelvisTrajectoryMessage);
   }

   public void computeFootTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeFootTrajectoryMessage(robotSide);
   }

   public void computeFootTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame footFrame = fullRobotModelToUseForConversion.getEndEffectorFrame(robotSide, LimbName.LEG);
      FramePose3D desiredFootPose = new FramePose3D(footFrame);
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      FootTrajectoryMessage footTrajectoryMessage = robotSide == RobotSide.LEFT ? output.getLeftFootTrajectoryMessage()
            : output.getRightFootTrajectoryMessage();
      footTrajectoryMessage.set(HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation));
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModelToUseForConversion;
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
