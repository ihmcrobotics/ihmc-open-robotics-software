package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StoredPropertySetMessage extends Packet<StoredPropertySetMessage> implements Settable<StoredPropertySetMessage>, EpsilonComparable<StoredPropertySetMessage>
{
   /**
            * StoredPropertySet values serialized as a string array
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  strings_;

   public StoredPropertySetMessage()
   {
      strings_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
   }

   public StoredPropertySetMessage(StoredPropertySetMessage other)
   {
      this();
      set(other);
   }

   public void set(StoredPropertySetMessage other)
   {
      strings_.set(other.strings_);
   }


   /**
            * StoredPropertySet values serialized as a string array
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getStrings()
   {
      return strings_;
   }


   public static Supplier<StoredPropertySetMessagePubSubType> getPubSubType()
   {
      return StoredPropertySetMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StoredPropertySetMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StoredPropertySetMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.strings_, other.strings_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StoredPropertySetMessage)) return false;

      StoredPropertySetMessage otherMyClass = (StoredPropertySetMessage) other;

      if (!this.strings_.equals(otherMyClass.strings_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StoredPropertySetMessage {");
      builder.append("strings=");
      builder.append(this.strings_);
      builder.append("}");
      return builder.toString();
   }
}
