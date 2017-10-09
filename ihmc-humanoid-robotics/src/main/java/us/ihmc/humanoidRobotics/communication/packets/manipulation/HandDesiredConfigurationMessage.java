package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "Packet for commanding the hands to perform various predefined grasps."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/hand_desired_configuration")
public class HandDesiredConfigurationMessage extends Packet<HandDesiredConfigurationMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies the grasp to perform")
   public HandConfiguration handDesiredConfiguration;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandDesiredConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Creates a message with the desired grasp to be performed.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide refers to which hand will perform the grasp.
    * @param handDesiredConfiguration refers to the desired grasp.
    */
   public HandDesiredConfigurationMessage(RobotSide robotSide, HandConfiguration handDesiredConfiguration)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.handDesiredConfiguration = handDesiredConfiguration;
   }

   public HandDesiredConfigurationMessage(Random random)
   {
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      handDesiredConfiguration = RandomNumbers.nextEnum(random, HandConfiguration.class);
   }

   public HandConfiguration getHandDesiredConfiguration()
   {
      return handDesiredConfiguration;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object other)
   {
      return ((other instanceof HandDesiredConfigurationMessage) && this.epsilonEquals((HandDesiredConfigurationMessage) other, 0));
   }

   @Override
   public String toString()
   {
      return robotSide.toString() + " State= " + handDesiredConfiguration.toString();
   }

   @Override
   public boolean epsilonEquals(HandDesiredConfigurationMessage other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.getHandDesiredConfiguration().equals(other.getHandDesiredConfiguration()));

      return ret;
   }
}
