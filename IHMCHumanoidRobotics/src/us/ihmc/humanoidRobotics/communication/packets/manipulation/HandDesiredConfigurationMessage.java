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
   public HandConfiguration fingerState;

   public HandDesiredConfigurationMessage()
   {
      // Empty constructor for deserialization
   }

   public HandDesiredConfigurationMessage(RobotSide robotSide, HandConfiguration fingerState)
   {
      this.robotSide = robotSide;
      this.fingerState = fingerState;
   }

   public HandConfiguration getFingerState()
   {
      return fingerState;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object obj)
   {
      return ((obj instanceof HandDesiredConfigurationMessage) && this.epsilonEquals((HandDesiredConfigurationMessage) obj, 0));
   }

   @Override
   public String toString()
   {
      return robotSide.toString() + " State= " + fingerState.toString();
   }

   @Override
   public boolean epsilonEquals(HandDesiredConfigurationMessage other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.getFingerState().equals(other.getFingerState()));

      return ret;
   }

   public HandDesiredConfigurationMessage(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, HandConfiguration.values()[random.nextInt(HandConfiguration.BASIC_GRIP.getDocumentedValues().length)]);

   }
}
