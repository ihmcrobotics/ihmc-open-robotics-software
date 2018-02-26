package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket>
{
   public static enum CurrentBehaviorStatus
   {
      NO_BEHAVIOR_RUNNING, BEHAVIOS_RUNNING, BEHAVIOR_PAUSED;
   }

   public CurrentBehaviorStatus currentStatus;

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

   public CurrentBehaviorStatus getCurrentStatus()
   {
      return currentStatus;
   }

   public boolean epsilonEquals(BehaviorStatusPacket other, double epsilon)
   {
      return this.currentStatus.equals(other.currentStatus);
   }

   
}
