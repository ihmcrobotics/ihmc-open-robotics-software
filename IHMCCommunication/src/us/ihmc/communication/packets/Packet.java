package us.ihmc.communication.packets;

import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.ComparableDataObject;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

public abstract class Packet<T> implements ComparableDataObject<T>
{
   public long uniqueId = 0; 
   
   @IgnoreField
   public byte destination = (byte) PacketDestination.BROADCAST.ordinal();
   
   @IgnoreField
   @Optional(value = "scripting")
   public String notes;
   
   
   public void setDestination(PacketDestination destination)
   {
      setDestination(destination.ordinal());
   }
   
   public void setDestination(int destination)
   {
      this.destination = (byte) destination;
   }
      
   public long getUniqueId()
   {
      return uniqueId;
   }

   public void setUniqueId(long uniqueId)
   {
      this.uniqueId = uniqueId;
   }

   public int getDestination()
   {
      return destination;
   }

   public boolean isClonable()
   {
      return true;
   }
   
   public void setNotes(String notes)
   {
      this.notes = notes;
   }
   
   public String getNotes()
   {
      return notes;
   }
}
