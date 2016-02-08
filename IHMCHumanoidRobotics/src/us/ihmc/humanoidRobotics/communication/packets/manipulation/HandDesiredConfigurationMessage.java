package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.FingerState;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation(value = "Packet for commanding the hands to perform various predefined grasps.")
public class HandDesiredConfigurationMessage extends IHMCRosApiPacket<HandDesiredConfigurationMessage>
{
   @FieldDocumentation(value = "Specifies the side of the robot that will execute the trajectory")
   public RobotSide robotSide;
   @FieldDocumentation(value = "Specifies the grasp to perform")
   public FingerState fingerState;

   public HandDesiredConfigurationMessage()
   {
      // Empty constructor for deserialization
   }

   public HandDesiredConfigurationMessage(RobotSide robotSide, FingerState fingerState)
   {
      this.robotSide = robotSide;
      this.fingerState = fingerState;
   }

   public FingerState getFingerState()
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
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, FingerState.values()[random.nextInt(FingerState.BASIC_GRIP.getDocumentedValues().length)]);

   }
}
