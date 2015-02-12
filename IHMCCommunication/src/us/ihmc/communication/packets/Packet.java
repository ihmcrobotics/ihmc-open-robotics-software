package us.ihmc.communication.packets;

import us.ihmc.utilities.ComparableDataObject;

public abstract class Packet<T> implements ComparableDataObject<T>
{
   private int destination = PacketDestination.BROADCAST.ordinal();
   public void setDestination(PacketDestination destination)
   {
      this.destination = destination.ordinal();
   }
   
   public void setDestination(int destination)
   {
      this.destination = destination;
   }
   
   public int getDestination()
   {
      return destination;
   }

   public boolean isClonable()
   {
      return true;
   }
}
