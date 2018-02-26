package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorTypePacket extends Packet<HumanoidBehaviorTypePacket>
{
   public HumanoidBehaviorType behaviorType;

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

   public HumanoidBehaviorType getBehaviorType()
   {
      return behaviorType;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorTypePacket other, double epsilon)
   {
      return this.getBehaviorType().equals(other.getBehaviorType());
   }
}
