package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket>
{
   public byte currentStatus;

   // empty constructor for deserialization
   public BehaviorStatusPacket()
   {
   }

   @Override
   public void set(BehaviorStatusPacket other)
   {
      setPacketInformation(other);
      currentStatus = other.currentStatus;
   }

   public byte getCurrentStatus()
   {
      return currentStatus;
   }

   public boolean epsilonEquals(BehaviorStatusPacket other, double epsilon)
   {
      return currentStatus == other.currentStatus;
   }
}
