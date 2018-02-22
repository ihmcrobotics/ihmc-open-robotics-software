package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket>
{
   public StringBuilder key = new StringBuilder();
   public double value;

   public SimpleCoactiveBehaviorDataPacket()
   {
   }

   @Override
   public void set(SimpleCoactiveBehaviorDataPacket other)
   {
      key.setLength(0);
      key.append(other.key);
      value = other.value;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      return key.equals(other.key) && value == other.value;
   }
}
