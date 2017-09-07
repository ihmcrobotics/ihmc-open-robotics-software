package us.ihmc.humanoidRobotics.communication.packets;


import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class BumStatePacket extends Packet<BumStatePacket>
{
   public boolean loadBearing;

   public BumStatePacket()
   {
      // Empty constructor for deserialization
   }

   public BumStatePacket(boolean loadBearing)
   {
      this.loadBearing = loadBearing;
   }

   public boolean isLoadBearing()
   {
      return loadBearing;
   }

   public void setLoadBearing(boolean loadBearing)
   {
      this.loadBearing = loadBearing;
   }

   public String toString()
   {
      return "Bum Load Bearing";
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof BumStatePacket) && this.epsilonEquals((BumStatePacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(BumStatePacket other, double epsilon)
   {
      return (this.isLoadBearing() == other.isLoadBearing());
   }

   public BumStatePacket(Random random)
   {
      this(random.nextBoolean());
   }
}
