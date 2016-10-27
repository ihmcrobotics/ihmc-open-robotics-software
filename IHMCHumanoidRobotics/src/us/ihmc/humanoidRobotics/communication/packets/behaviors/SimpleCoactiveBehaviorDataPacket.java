package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket>
{
   public String key;
   public double data;
   

   public SimpleCoactiveBehaviorDataPacket()
   {
      
   }
   
   public SimpleCoactiveBehaviorDataPacket(String key, double data)
   {
      this.key = key;
      this.data = data;

   }

   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      return key.equals(other.key)&& data == other.data;
   }
   
}
