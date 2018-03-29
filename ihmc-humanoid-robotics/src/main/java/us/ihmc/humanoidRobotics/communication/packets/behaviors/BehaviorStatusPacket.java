package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket>
{
   public static final byte NO_BEHAVIOR_RUNNING = 0;
   public static final byte BEHAVIOS_RUNNING = 1;
   public static final byte BEHAVIOR_PAUSED = 2;

   public byte currentBehaviorStatus;

   // empty constructor for deserialization
   public BehaviorStatusPacket()
   {
   }

   @Override
   public void set(BehaviorStatusPacket other)
   {
      setPacketInformation(other);
      currentBehaviorStatus = other.currentBehaviorStatus;
   }

   public byte getCurrentBehaviorStatus()
   {
      return currentBehaviorStatus;
   }

   public boolean epsilonEquals(BehaviorStatusPacket other, double epsilon)
   {
      return currentBehaviorStatus == other.currentBehaviorStatus;
   }
}
