package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.\n"
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule DOES apply to the fields of this message."
      + " If setting a field to null is not an option (going through IHMC ROS API), the user can use the latter rule to select the messages to be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/whole_body_trajectory")
public class WholeBodyTrajectoryMessage extends Packet<WholeBodyTrajectoryMessage>
{
   @RosExportedField(documentation = "Trajectory for the left hand")
   public HandTrajectoryMessage leftHandTrajectoryMessage = new HandTrajectoryMessage();
   @RosExportedField(documentation = "Trajectory for the right hand")
   public HandTrajectoryMessage rightHandTrajectoryMessage = new HandTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the left arm joints")
   public ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage();
   @RosExportedField(documentation = "Trajectory for the right arm joints")
   public ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the chest")
   public ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the pelvis")
   public PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the left foot")
   public FootTrajectoryMessage leftFootTrajectoryMessage = new FootTrajectoryMessage();
   @RosExportedField(documentation = "Trajectory for the right foot")
   public FootTrajectoryMessage rightFootTrajectoryMessage = new FootTrajectoryMessage();

   @RosExportedField(documentation = "Trajectory for the head")
   public HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage();

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public WholeBodyTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage other)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      leftHandTrajectoryMessage = new HandTrajectoryMessage(other.leftHandTrajectoryMessage);
      rightHandTrajectoryMessage = new HandTrajectoryMessage(other.rightHandTrajectoryMessage);
      leftArmTrajectoryMessage = new ArmTrajectoryMessage(other.leftArmTrajectoryMessage);
      rightArmTrajectoryMessage = new ArmTrajectoryMessage(other.rightArmTrajectoryMessage);
      chestTrajectoryMessage = new ChestTrajectoryMessage(other.chestTrajectoryMessage);
      pelvisTrajectoryMessage = new PelvisTrajectoryMessage(other.pelvisTrajectoryMessage);
      leftFootTrajectoryMessage = new FootTrajectoryMessage(other.leftFootTrajectoryMessage);
      rightFootTrajectoryMessage = new FootTrajectoryMessage(other.rightFootTrajectoryMessage);
      headTrajectoryMessage = new HeadTrajectoryMessage(other.headTrajectoryMessage);
   }

   @Override
   public void set(WholeBodyTrajectoryMessage other)
   {
      leftHandTrajectoryMessage.set(other.leftHandTrajectoryMessage);
      rightHandTrajectoryMessage.set(other.rightHandTrajectoryMessage);
      leftArmTrajectoryMessage.set(other.leftArmTrajectoryMessage);
      rightArmTrajectoryMessage.set(other.rightArmTrajectoryMessage);
      chestTrajectoryMessage.set(other.chestTrajectoryMessage);
      pelvisTrajectoryMessage.set(other.pelvisTrajectoryMessage);
      leftFootTrajectoryMessage.set(other.leftFootTrajectoryMessage);
      rightFootTrajectoryMessage.set(other.rightFootTrajectoryMessage);
      headTrajectoryMessage.set(other.headTrajectoryMessage);
      setPacketInformation(other);
   }

   public ChestTrajectoryMessage getChestTrajectoryMessage()
   {
      return chestTrajectoryMessage;
   }

   public PelvisTrajectoryMessage getPelvisTrajectoryMessage()
   {
      return pelvisTrajectoryMessage;
   }

   public HeadTrajectoryMessage getHeadTrajectoryMessage()
   {
      return headTrajectoryMessage;
   }

   public void setLeftHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      leftHandTrajectoryMessage.set(handTrajectoryMessage);
   }

   public void setRightHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      rightHandTrajectoryMessage.set(handTrajectoryMessage);
   }

   public void setLeftArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      leftArmTrajectoryMessage.set(armTrajectoryMessage);
   }

   public void setRightArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      rightArmTrajectoryMessage.set(armTrajectoryMessage);
   }

   public void setChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      this.chestTrajectoryMessage.set(chestTrajectoryMessage);
   }

   public void setPelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      this.pelvisTrajectoryMessage.set(pelvisTrajectoryMessage);
   }

   public void setLeftFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      leftFootTrajectoryMessage.set(footTrajectoryMessage);
   }

   public void setRightFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      rightFootTrajectoryMessage.set(footTrajectoryMessage);
   }

   public void setHeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage)
   {
      this.headTrajectoryMessage.set(headTrajectoryMessage);
   }

   public HandTrajectoryMessage getLeftHandTrajectoryMessage()
   {
      return leftHandTrajectoryMessage;
   }

   public HandTrajectoryMessage getRightHandTrajectoryMessage()
   {
      return rightHandTrajectoryMessage;
   }

   public ArmTrajectoryMessage getLeftArmTrajectoryMessage()
   {
      return leftArmTrajectoryMessage;
   }

   public ArmTrajectoryMessage getRightArmTrajectoryMessage()
   {
      return rightArmTrajectoryMessage;
   }

   public FootTrajectoryMessage getLeftFootTrajectoryMessage()
   {
      return leftFootTrajectoryMessage;
   }

   public FootTrajectoryMessage getRightFootTrajectoryMessage()
   {
      return rightFootTrajectoryMessage;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * 
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      executionDelayTime = delayTime;
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
      if (!headTrajectoryMessage.epsilonEquals(other.headTrajectoryMessage, epsilon))
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

      if (leftHandTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !leftHandTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = leftHandTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
         else if (RobotSide.fromByte(leftHandTrajectoryMessage.getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftHandTrajectoryMessage field is inconsistent with its name.";
      }
      if (rightHandTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !leftHandTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = rightHandTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
         else if (RobotSide.fromByte(rightHandTrajectoryMessage.getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightHandTrajectoryMessage field is inconsistent with its name.";
      }
      if (leftArmTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !leftArmTrajectoryMessage.jointspaceTrajectory.jointTrajectoryMessages.isEmpty())
      {
         if ((errorMessage = leftArmTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
         else if (RobotSide.fromByte(leftArmTrajectoryMessage.getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftArmTrajectoryMessage field is inconsistent with its name.";
      }
      if (rightArmTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !rightArmTrajectoryMessage.jointspaceTrajectory.jointTrajectoryMessages.isEmpty())
      {
         if ((errorMessage = rightArmTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
         else if (RobotSide.fromByte(rightArmTrajectoryMessage.getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightArmTrajectoryMessage field is inconsistent with its name.";
      }
      if (chestTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !chestTrajectoryMessage.so3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = chestTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
      }
      if (pelvisTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !pelvisTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = pelvisTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
      }
      if (headTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !headTrajectoryMessage.so3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = headTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
      }
      if (leftFootTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !leftFootTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = leftFootTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
         else if (RobotSide.fromByte(leftFootTrajectoryMessage.getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftFootTrajectoryMessage field is inconsistent with its name.";
      }
      if (rightFootTrajectoryMessage.getUniqueId() != INVALID_MESSAGE_ID && !rightFootTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = rightFootTrajectoryMessage.validateMessage()) != null)
            return errorMessage;
         else if (RobotSide.fromByte(rightFootTrajectoryMessage.getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightFootTrajectoryMessage field is inconsistent with its name.";
      }

      return null;
   }

   @Override
   public String toString()
   {
      String string = getClass().getSimpleName() + ":";
      string += "\n" + leftHandTrajectoryMessage.toString();
      string += "\n" + rightHandTrajectoryMessage.toString();
      string += "\n" + leftArmTrajectoryMessage.toString();
      string += "\n" + rightArmTrajectoryMessage.toString();
      string += "\n" + pelvisTrajectoryMessage.toString();
      string += "\n" + headTrajectoryMessage.toString();
      string += "\n" + leftFootTrajectoryMessage.toString();
      string += "\n" + rightFootTrajectoryMessage.toString();
      return string;
   }
}
