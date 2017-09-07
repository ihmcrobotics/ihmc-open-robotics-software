package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket>
{
   public static enum CurrentBehaviorStatus
   {
      NO_BEHAVIOR_RUNNING, BEHAVIOS_RUNNING, BEHAVIOR_PAUSED;
   }

   private CurrentBehaviorStatus currentStatus;

   // empty constructor for deserialization
   public BehaviorStatusPacket()
   {
   }

   public BehaviorStatusPacket(CurrentBehaviorStatus requestedControl)
   {
      this.currentStatus = requestedControl;
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
