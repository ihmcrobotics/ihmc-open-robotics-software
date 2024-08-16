package us.ihmc.avatar.testTools;

import java.util.Random;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class AvatarRandomTestMessages
{
   public static ArmTrajectoryMessage nextArmTrajectoryMessage(Random random, double trajectoryTime, RobotSide robotSide, FullHumanoidRobotModel robot)
   {
      RigidBodyBasics chest = robot.getChest();
      RigidBodyBasics hand = robot.getHand(robotSide);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);

      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
      double[] desiredJointPositions = new double[numberOfJoints];
      for (int i = 0; i < armJoints.length; i++)
      {
         desiredJointPositions[i] = RandomNumbers.nextDouble(random, armJoints[i].getJointLimitLower(), armJoints[i].getJointLimitUpper());
      }

      return HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
   }

   public static ChestTrajectoryMessage nextChestTrajectoryMessage(Random random, double trajectoryTime, ReferenceFrame trajectoryFrame,
                                                                   FullHumanoidRobotModel robot)
   {
      RigidBodyBasics pelvis = robot.getPelvis();
      RigidBodyBasics chest = robot.getChest();
      OneDoFJointBasics[] spineClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);

      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);
      
      spineClone[0].updateFramesRecursively();
      RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredOrientation.changeFrame(trajectoryFrame);

      return HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
   }

   public static PelvisTrajectoryMessage nextPelvisTrajectoryMessage(Random random, double trajectoryTime, FullHumanoidRobotModel robot, double maxTranslation,
                                                                     double maxRotation)
   {
      MovingReferenceFrame pelvisFrame = robot.getPelvis().getBodyFixedFrame();

      double translation = EuclidCoreRandomTools.nextDouble(random, maxTranslation);
      Vector3D translationInPelvis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, translation);
      FramePoint3D desiredPosition = new FramePoint3D(pelvisFrame, translationInPelvis);
      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion rotationInPelvis = EuclidCoreRandomTools.nextQuaternion(random, maxRotation);
      FrameQuaternion desiredOrientation = new FrameQuaternion(pelvisFrame, rotationInPelvis);
      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      return HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);
   }
}
