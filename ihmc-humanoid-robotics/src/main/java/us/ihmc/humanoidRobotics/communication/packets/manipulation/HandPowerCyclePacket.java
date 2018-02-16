package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class HandPowerCyclePacket extends Packet<HandPowerCyclePacket>
{
   public byte robotSide;

   public HandPowerCyclePacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      // Empty constructor for deserialization
   }

   @Override
   public void set(HandPowerCyclePacket other)
   {
      robotSide = other.robotSide;
      setPacketInformation(other);
   }

   public byte getRobotSide()
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
      return robotSide == other.robotSide;
   }
}
