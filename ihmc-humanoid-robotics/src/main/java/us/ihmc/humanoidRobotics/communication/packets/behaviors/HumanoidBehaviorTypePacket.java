package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorTypePacket extends Packet<HumanoidBehaviorTypePacket>
{
   public byte humanoidBehaviorType;

   // empty constructor for deserialization
   public HumanoidBehaviorTypePacket()
   {
   }

   @Override
   public void set(HumanoidBehaviorTypePacket other)
   {
      humanoidBehaviorType = other.humanoidBehaviorType;
      setPacketInformation(other);
   }

   public byte getBehaviorType()
   {
      return humanoidBehaviorType;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorTypePacket other, double epsilon)
   {
      return humanoidBehaviorType == other.humanoidBehaviorType;
   }
}
