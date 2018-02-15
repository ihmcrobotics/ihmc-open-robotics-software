package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket>
{
   public String key;
   public double value;

   public SimpleCoactiveBehaviorDataPacket()
   {
   }

   public SimpleCoactiveBehaviorDataPacket(String key, double value)
   {
      this.key = key;
      this.value = value;
   }

   @Override
   public void set(SimpleCoactiveBehaviorDataPacket other)
   {
      key = other.key;
      value = other.value;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      return key.equals(other.key) && value == other.value;
   }
}
