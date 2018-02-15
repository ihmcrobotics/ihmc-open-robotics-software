package us.ihmc.humanoidRobotics.communication.packets.manipulation;

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

   @Override
   public void set(HandPowerCyclePacket other)
   {
      robotSide = other.robotSide;
      setPacketInformation(other);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object obj)
   {
      return obj instanceof HandPowerCyclePacket && epsilonEquals((HandPowerCyclePacket) obj, 0);
   }

   @Override
   public boolean epsilonEquals(HandPowerCyclePacket other, double epsilon)
   {
      boolean ret = getRobotSide().equals(other.getRobotSide());

      return ret;
   }
}
