package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorTypePacket extends Packet<HumanoidBehaviorTypePacket>
{
   public byte behaviorType;

   // empty constructor for deserialization
   public HumanoidBehaviorTypePacket()
   {
   }

   @Override
   public void set(HumanoidBehaviorTypePacket other)
   {
      behaviorType = other.behaviorType;
      setPacketInformation(other);
   }

   public byte getBehaviorType()
   {
      return behaviorType;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorTypePacket other, double epsilon)
   {
      return behaviorType == other.behaviorType;
   }
}
