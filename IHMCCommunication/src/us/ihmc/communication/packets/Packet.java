package us.ihmc.communication.packets;

import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.utilities.ComparableDataObject;

public abstract class Packet<T> implements ComparableDataObject<T>
{
   @IgnoreField
   public byte destination = (byte) PacketDestination.BROADCAST.ordinal();
   public void setDestination(PacketDestination destination)
   {
      setDestination(destination.ordinal());
   }
   
   public void setDestination(int destination)
   {
      this.destination = (byte) destination;
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
