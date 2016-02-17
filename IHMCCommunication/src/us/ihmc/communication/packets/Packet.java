package us.ihmc.communication.packets;

import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.ComparableDataObject;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

public abstract class Packet<T> implements ComparableDataObject<T>
{
   @FieldDocumentation("A unique id for the current message. This can be a timestamp or sequence number.\n"
         + "Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n"
         + "Use /output/last_received_message for feedback about when the last message was received.\n"
         + "A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.")
   public long uniqueId = 0; 

   @IgnoreField
   public static final long INVALID_MESSAGE_ID = 0L;

   @IgnoreField
   public static final long VALID_MESSAGE_DEFAULT_ID = -1L;

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
