package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandPowerCyclePacket extends Packet<HandPowerCyclePacket>
{
   public RobotSide robotSide;

   public HandPowerCyclePacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      // Empty constructor for deserialization
   }

   public HandPowerCyclePacket(RobotSide robotSide)
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof HandPowerCyclePacket) && this.epsilonEquals((HandPowerCyclePacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(HandPowerCyclePacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());

      return ret;
   }

   public HandPowerCyclePacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT);

   }
}
