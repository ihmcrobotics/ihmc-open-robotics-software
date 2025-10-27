package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class YoRegistryMessage extends Packet<YoRegistryMessage> implements Settable<YoRegistryMessage>, EpsilonComparable<YoRegistryMessage>
{
   /**
            * Depth first ordered uncompressed long bits variables
            */
   public us.ihmc.idl.IDLSequence.Byte  data_;

   public YoRegistryMessage()
   {
      data_ = new us.ihmc.idl.IDLSequence.Byte (10000, "type_9");

   }

   public YoRegistryMessage(YoRegistryMessage other)
   {
      this();
      set(other);
   }

   public void set(YoRegistryMessage other)
   {
      data_.set(other.data_);
   }


   /**
            * Depth first ordered uncompressed long bits variables
            */
   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }


   public static Supplier<YoRegistryMessagePubSubType> getPubSubType()
   {
      return YoRegistryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YoRegistryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YoRegistryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YoRegistryMessage)) return false;

      YoRegistryMessage otherMyClass = (YoRegistryMessage) other;

      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YoRegistryMessage {");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
