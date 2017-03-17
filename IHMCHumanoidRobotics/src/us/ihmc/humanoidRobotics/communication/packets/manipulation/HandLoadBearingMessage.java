package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.humanoidRobotics.communication.packets.AbstractLoadBearingMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingMessage extends AbstractLoadBearingMessage<HandLoadBearingMessage>
{
   public RobotSide robotSide;

   public HandLoadBearingMessage()
   {
      super();
   }

   public HandLoadBearingMessage(RobotSide robotSide)
   {
      super();
      this.robotSide = robotSide;
   }

   public HandLoadBearingMessage(Random random)
   {
      super(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
   }

   @Override
   public boolean epsilonEquals(HandLoadBearingMessage other, double epsilon)
   {
      return robotSide == other.robotSide && super.epsilonEquals(other, epsilon);
   }
}
