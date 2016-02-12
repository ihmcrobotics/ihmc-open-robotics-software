package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.\n"
      + "When going through our ROS API: set the id to 0 (zero) any message that should not be processed by the controller, set the field to null when going through our Java API.")
public class WholeBodyTrajectoryMessage extends IHMCRosApiPacket<WholeBodyTrajectoryMessage>
      implements VisualizablePacket, TransformableDataObject<WholeBodyTrajectoryMessage>
{
   public HandTrajectoryMessage leftHandTrajectoryMessage, rightHandTrajectoryMessage;
   public ArmTrajectoryMessage leftArmTrajectoryMessage, rightArmTrajectoryMessage;
   public ChestTrajectoryMessage chestTrajectoryMessage;
   public PelvisTrajectoryMessage pelvisTrajectoryMessage;
   public FootTrajectoryMessage leftFootTrajectoryMessage, rightFootTrajectoryMessage;

   public WholeBodyTrajectoryMessage()
   {
   }

   public HandTrajectoryMessage getHandTrajectoryMessage(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         return leftHandTrajectoryMessage;
      case RIGHT:
         return rightHandTrajectoryMessage;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public ArmTrajectoryMessage getArmTrajectoryMessage(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         return leftArmTrajectoryMessage;
      case RIGHT:
         return rightArmTrajectoryMessage;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public ChestTrajectoryMessage getChestTrajectoryMessage()
   {
      return chestTrajectoryMessage;
   }

   public PelvisTrajectoryMessage getPelvisTrajectoryMessage()
   {
      return pelvisTrajectoryMessage;
   }

   public FootTrajectoryMessage getFootTrajectoryMessage(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         return leftFootTrajectoryMessage;
      case RIGHT:
         return rightFootTrajectoryMessage;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public void setHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      switch (handTrajectoryMessage.getRobotSide())
      {
      case LEFT:
         leftHandTrajectoryMessage = handTrajectoryMessage;
      case RIGHT:
         rightHandTrajectoryMessage = handTrajectoryMessage;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public void setArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      switch (armTrajectoryMessage.getRobotSide())
      {
      case LEFT:
         leftArmTrajectoryMessage = armTrajectoryMessage;
      case RIGHT:
         rightArmTrajectoryMessage = armTrajectoryMessage;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public void setChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      this.chestTrajectoryMessage = chestTrajectoryMessage;
   }

   public void setPelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      this.pelvisTrajectoryMessage = pelvisTrajectoryMessage;
   }

   public void setFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      switch (footTrajectoryMessage.getRobotSide())
      {
      case LEFT:
         leftFootTrajectoryMessage = footTrajectoryMessage;
      case RIGHT:
         rightFootTrajectoryMessage = footTrajectoryMessage;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public boolean checkRobotSideConsistency()
   {
      if (leftHandTrajectoryMessage.getRobotSide() != RobotSide.LEFT)
         return false;
      if (rightHandTrajectoryMessage.getRobotSide() != RobotSide.RIGHT)
         return false;
      if (leftArmTrajectoryMessage.getRobotSide() != RobotSide.LEFT)
         return false;
      if (rightArmTrajectoryMessage.getRobotSide() != RobotSide.RIGHT)
         return false;
      if (leftFootTrajectoryMessage.getRobotSide() != RobotSide.LEFT)
         return false;
      if (rightFootTrajectoryMessage.getRobotSide() != RobotSide.RIGHT)
         return false;

      return true;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryMessage other, double epsilon)
   {
      if (!leftHandTrajectoryMessage.epsilonEquals(other.leftHandTrajectoryMessage, epsilon))
         return false;
      if (!rightHandTrajectoryMessage.epsilonEquals(other.rightHandTrajectoryMessage, epsilon))
         return false;
      if (!leftArmTrajectoryMessage.epsilonEquals(other.leftArmTrajectoryMessage, epsilon))
         return false;
      if (!rightArmTrajectoryMessage.epsilonEquals(other.rightArmTrajectoryMessage, epsilon))
         return false;
      if (!chestTrajectoryMessage.epsilonEquals(other.chestTrajectoryMessage, epsilon))
         return false;
      if (!pelvisTrajectoryMessage.epsilonEquals(other.pelvisTrajectoryMessage, epsilon))
         return false;
      if (!leftFootTrajectoryMessage.epsilonEquals(other.leftFootTrajectoryMessage, epsilon))
         return false;
      if (!rightFootTrajectoryMessage.epsilonEquals(other.rightFootTrajectoryMessage, epsilon))
         return false;

      return true;
   }

   @Override
   public WholeBodyTrajectoryMessage transform(RigidBodyTransform transform)
   {
      WholeBodyTrajectoryMessage transformedWholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      transformedWholeBodyTrajectoryMessage.leftHandTrajectoryMessage = leftHandTrajectoryMessage.transform(transform);
      transformedWholeBodyTrajectoryMessage.rightHandTrajectoryMessage = rightHandTrajectoryMessage.transform(transform);
      transformedWholeBodyTrajectoryMessage.leftArmTrajectoryMessage = new ArmTrajectoryMessage(leftArmTrajectoryMessage);
      transformedWholeBodyTrajectoryMessage.rightArmTrajectoryMessage = new ArmTrajectoryMessage(rightArmTrajectoryMessage);
      transformedWholeBodyTrajectoryMessage.chestTrajectoryMessage = chestTrajectoryMessage.transform(transform);
      transformedWholeBodyTrajectoryMessage.pelvisTrajectoryMessage = pelvisTrajectoryMessage.transform(transform);
      transformedWholeBodyTrajectoryMessage.leftFootTrajectoryMessage = leftFootTrajectoryMessage.transform(transform);
      transformedWholeBodyTrajectoryMessage.rightFootTrajectoryMessage = rightFootTrajectoryMessage.transform(transform);

      return transformedWholeBodyTrajectoryMessage;
   }

}
