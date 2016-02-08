package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation(value = "Packet for commanding the hands to perform various predefined grasps.")
public class HandDesiredConfigurationMessage extends IHMCRosApiPacket<HandDesiredConfigurationMessage>
{
   @FieldDocumentation(value = "Specifies the side of the robot that will execute the trajectory")
   public RobotSide robotSide;
   @FieldDocumentation(value = "Specifies the grasp to perform")
   public HandConfiguration handDesiredConfiguration;

   public HandDesiredConfigurationMessage()
   {
      // Empty constructor for deserialization
   }

   public HandDesiredConfigurationMessage(RobotSide robotSide, HandConfiguration handDesiredConfiguration)
   {
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
