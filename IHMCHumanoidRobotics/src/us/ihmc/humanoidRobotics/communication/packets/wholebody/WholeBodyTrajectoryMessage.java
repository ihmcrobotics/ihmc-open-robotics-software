package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.communication.packets.MultiplePacketHolder;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.\n"
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule DOES apply to the fields of this message."
      + " If setting a field to null is not an option (going through IHMC ROS API), the user can use the latter rule to select the messages to be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/whole_body_trajectory")
public class WholeBodyTrajectoryMessage extends TrackablePacket<WholeBodyTrajectoryMessage> implements VisualizablePacket, MultiplePacketHolder
{
   @RosExportedField(documentation = "Trajectory for the left hand")
   public HandTrajectoryMessage leftHandTrajectoryMessage;
   @RosExportedField(documentation = "Trajectory for the right hand")
   public HandTrajectoryMessage rightHandTrajectoryMessage;

   @RosExportedField(documentation = "Trajectory for the left arm joints")
   public ArmTrajectoryMessage leftArmTrajectoryMessage;
   @RosExportedField(documentation = "Trajectory for the right arm joints")
   public ArmTrajectoryMessage rightArmTrajectoryMessage;

   @RosExportedField(documentation = "Trajectory for the chest")
   public ChestTrajectoryMessage chestTrajectoryMessage;

   @RosExportedField(documentation = "Trajectory for the pelvis")
   public PelvisTrajectoryMessage pelvisTrajectoryMessage;

   @RosExportedField(documentation = "Trajectory for the left foot")
   public FootTrajectoryMessage leftFootTrajectoryMessage;
   @RosExportedField(documentation = "Trajectory for the right foot")
   public FootTrajectoryMessage rightFootTrajectoryMessage;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public WholeBodyTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryMessage(Random random)
   {
      leftHandTrajectoryMessage = new HandTrajectoryMessage(random);
      leftHandTrajectoryMessage.robotSide = RobotSide.LEFT;

      rightHandTrajectoryMessage = new HandTrajectoryMessage(random);
      rightHandTrajectoryMessage.robotSide = RobotSide.RIGHT;

      leftArmTrajectoryMessage = new ArmTrajectoryMessage(random);
      leftArmTrajectoryMessage.robotSide = RobotSide.LEFT;

      rightArmTrajectoryMessage = new ArmTrajectoryMessage(random);
      rightArmTrajectoryMessage.robotSide = RobotSide.RIGHT;

      leftFootTrajectoryMessage = new FootTrajectoryMessage(random);
      leftFootTrajectoryMessage.robotSide = RobotSide.LEFT;

      rightFootTrajectoryMessage = new FootTrajectoryMessage(random);
      rightFootTrajectoryMessage.robotSide = RobotSide.RIGHT;

      chestTrajectoryMessage = new ChestTrajectoryMessage(random);
      pelvisTrajectoryMessage = new PelvisTrajectoryMessage(random);
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
         return;
      case RIGHT:
         rightHandTrajectoryMessage = handTrajectoryMessage;
         return;
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
         return;
      case RIGHT:
         rightArmTrajectoryMessage = armTrajectoryMessage;
         return;
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
         return;
      case RIGHT:
         rightFootTrajectoryMessage = footTrajectoryMessage;
         return;
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

   public void clear()
   {
      clearHandTrajectoryMessages();
      clearArmTrajectoryMessages();
      clearChestTrajectoryMessage();
      clearPelvisTrajectoryMessage();
      clearFootTrajectoryMessages();
   }

   public void clearHandTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         clearHandTrajectoryMessage(robotSide);
   }

   public void clearHandTrajectoryMessage(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         leftHandTrajectoryMessage = null;
         return;
      case RIGHT:
         rightHandTrajectoryMessage = null;
         return;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public void clearArmTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         clearArmTrajectoryMessage(robotSide);
   }

   public void clearArmTrajectoryMessage(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         leftArmTrajectoryMessage = null;
         return;
      case RIGHT:
         rightArmTrajectoryMessage = null;
         return;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public void clearChestTrajectoryMessage()
   {
      chestTrajectoryMessage = null;
   }

   public void clearPelvisTrajectoryMessage()
   {
      pelvisTrajectoryMessage = null;
   }

   public void clearFootTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         clearFootTrajectoryMessage(robotSide);
   }

   public void clearFootTrajectoryMessage(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         leftFootTrajectoryMessage = null;
         return;
      case RIGHT:
         rightFootTrajectoryMessage = null;
         return;
      default:
         throw new RuntimeException("Should not get there.");
      }
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

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      String errorMessage = PacketValidityChecker.validatePacket(this, true);
      if (errorMessage != null)
         return errorMessage;
      if (!checkRobotSideConsistency())
      {
         errorMessage = "The robotSide of a field is inconsistent with its name.";
         return errorMessage;
      }

      if ((errorMessage = validateIfNeeded(leftHandTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(rightHandTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(leftArmTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(rightArmTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(chestTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(pelvisTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(leftFootTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(rightFootTrajectoryMessage)) != null)
         return errorMessage;

      return null;
   }

   private String validateIfNeeded(Packet<?> message)
   {
      String errorMessage = null;

      if (message != null && message.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         errorMessage = message.validateMessage();

      return errorMessage;
   }

   @Override
   public List<Packet<?>> getPackets()
   {
      ArrayList<Packet<?>> wholeBodyPackets = new ArrayList<>();
      wholeBodyPackets.add(leftHandTrajectoryMessage);
      wholeBodyPackets.add(rightHandTrajectoryMessage);
      wholeBodyPackets.add(leftArmTrajectoryMessage);
      wholeBodyPackets.add(rightArmTrajectoryMessage);
      wholeBodyPackets.add(chestTrajectoryMessage);
      wholeBodyPackets.add(pelvisTrajectoryMessage);
      wholeBodyPackets.add(leftFootTrajectoryMessage);
      wholeBodyPackets.add(rightFootTrajectoryMessage);
      return wholeBodyPackets;
   }
}
