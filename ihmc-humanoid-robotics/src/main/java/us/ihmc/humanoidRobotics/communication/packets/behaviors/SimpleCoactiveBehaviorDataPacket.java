package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket>
{
   public String key;
   public double value;
   public Object dataObject;
   

   public SimpleCoactiveBehaviorDataPacket()
   {
      
   }
   
   public SimpleCoactiveBehaviorDataPacket(String key, double value)
   {
      this.key = key;
      this.value = value;

   }

   public SimpleCoactiveBehaviorDataPacket(String key, Object dataObject)
   {
      this.key = key;
      this.dataObject = dataObject;

   }
   
   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      return key.equals(other.key)&& value == other.value;
   }
   
}
