package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation(value = "Packet for commanding the hands to perform various predefined grasps."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.")
public class HandDesiredConfigurationMessage extends IHMCRosApiPacket<HandDesiredConfigurationMessage>
{
   @FieldDocumentation(value = "Specifies the side of the robot that will execute the trajectory")
   public RobotSide robotSide;
   @FieldDocumentation(value = "Specifies the grasp to perform")
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

   public HandDesiredConfigurationMessage(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, HandConfiguration.values()[random.nextInt(HandConfiguration.BASIC_GRIP.getDocumentedValues().length)]);

   }
}
