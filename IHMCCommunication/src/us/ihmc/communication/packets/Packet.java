package us.ihmc.communication.packets;

import us.ihmc.utilities.net.ComparableDataObject;

public abstract class Packet<T> implements ComparableDataObject<T>
{
   private PacketDestination destination;
   
   public void setDestination(PacketDestination destination)
   {
      this.destination = destination;
   }
   
   public PacketDestination getDestination()
   {
      return destination;
   }
}
