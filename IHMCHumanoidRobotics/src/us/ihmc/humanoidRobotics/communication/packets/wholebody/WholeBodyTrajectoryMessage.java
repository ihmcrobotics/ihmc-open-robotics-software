package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
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
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule DOES apply to the fields of this message."
      + " If setting a field to null is not an option (going through IHMC ROS API), the user can use the latter rule to select the messages to be processed by the controller.")
public class WholeBodyTrajectoryMessage extends IHMCRosApiMessage<WholeBodyTrajectoryMessage>
      implements VisualizablePacket, TransformableDataObject<WholeBodyTrajectoryMessage>
{
   public HandTrajectoryMessage leftHandTrajectoryMessage, rightHandTrajectoryMessage;
   public ArmTrajectoryMessage leftArmTrajectoryMessage, rightArmTrajectoryMessage;
   public ChestTrajectoryMessage chestTrajectoryMessage;
   public PelvisTrajectoryMessage pelvisTrajectoryMessage;
   public FootTrajectoryMessage leftFootTrajectoryMessage, rightFootTrajectoryMessage;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public WholeBodyTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
      if (handTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
         handTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);

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
      if (armTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
         armTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);

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
      if (chestTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
         chestTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      this.chestTrajectoryMessage = chestTrajectoryMessage;
   }

   public void setPelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      if (pelvisTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
         pelvisTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      this.pelvisTrajectoryMessage = pelvisTrajectoryMessage;
   }

   public void setFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      if (footTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
         footTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);

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
      if (leftHandTrajectoryMessage != null && leftHandTrajectoryMessage.getRobotSide() != RobotSide.LEFT)
         return false;
      if (rightHandTrajectoryMessage != null && rightHandTrajectoryMessage.getRobotSide() != RobotSide.RIGHT)
         return false;
      if (leftArmTrajectoryMessage != null && leftArmTrajectoryMessage.getRobotSide() != RobotSide.LEFT)
         return false;
      if (rightArmTrajectoryMessage != null && rightArmTrajectoryMessage.getRobotSide() != RobotSide.RIGHT)
         return false;
      if (leftFootTrajectoryMessage != null && leftFootTrajectoryMessage.getRobotSide() != RobotSide.LEFT)
         return false;
      if (rightFootTrajectoryMessage != null && rightFootTrajectoryMessage.getRobotSide() != RobotSide.RIGHT)
         return false;

      return true;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryMessage other, double epsilon)
   {
      if (leftHandTrajectoryMessage == null && other.leftHandTrajectoryMessage != null)
         return false;
      if (rightHandTrajectoryMessage == null && other.rightHandTrajectoryMessage != null)
         return false;
      if (leftArmTrajectoryMessage == null && other.leftArmTrajectoryMessage != null)
         return false;
      if (rightArmTrajectoryMessage == null && other.rightArmTrajectoryMessage != null)
         return false;
      if (chestTrajectoryMessage == null && other.chestTrajectoryMessage != null)
         return false;
      if (pelvisTrajectoryMessage == null && other.pelvisTrajectoryMessage != null)
         return false;
      if (leftFootTrajectoryMessage == null && other.leftFootTrajectoryMessage != null)
         return false;
      if (rightFootTrajectoryMessage == null && other.rightFootTrajectoryMessage != null)
         return false;

      if (leftHandTrajectoryMessage != null && other.leftHandTrajectoryMessage == null)
         return false;
      if (rightHandTrajectoryMessage != null && other.rightHandTrajectoryMessage == null)
         return false;
      if (leftArmTrajectoryMessage != null && other.leftArmTrajectoryMessage == null)
         return false;
      if (rightArmTrajectoryMessage != null && other.rightArmTrajectoryMessage == null)
         return false;
      if (chestTrajectoryMessage != null && other.chestTrajectoryMessage == null)
         return false;
      if (pelvisTrajectoryMessage != null && other.pelvisTrajectoryMessage == null)
         return false;
      if (leftFootTrajectoryMessage != null && other.leftFootTrajectoryMessage == null)
         return false;
      if (rightFootTrajectoryMessage != null && other.rightFootTrajectoryMessage == null)
         return false;

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

      if (leftHandTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.leftHandTrajectoryMessage = leftHandTrajectoryMessage.transform(transform);
      if (rightHandTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.rightHandTrajectoryMessage = rightHandTrajectoryMessage.transform(transform);
      if (leftArmTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.leftArmTrajectoryMessage = new ArmTrajectoryMessage(leftArmTrajectoryMessage);
      if (rightArmTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.rightArmTrajectoryMessage = new ArmTrajectoryMessage(rightArmTrajectoryMessage);
      if (chestTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.chestTrajectoryMessage = chestTrajectoryMessage.transform(transform);
      if (pelvisTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.pelvisTrajectoryMessage = pelvisTrajectoryMessage.transform(transform);
      if (leftFootTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.leftFootTrajectoryMessage = leftFootTrajectoryMessage.transform(transform);
      if (rightFootTrajectoryMessage != null)
         transformedWholeBodyTrajectoryMessage.rightFootTrajectoryMessage = rightFootTrajectoryMessage.transform(transform);

      return transformedWholeBodyTrajectoryMessage;
   }
}
