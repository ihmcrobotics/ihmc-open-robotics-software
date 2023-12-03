package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Holds a UUID
       */
public class UUIDMessage extends Packet<UUIDMessage> implements Settable<UUIDMessage>, EpsilonComparable<UUIDMessage>
{
   public long least_significant_bits_;
   public long most_significant_bits_;

   public UUIDMessage()
   {
   }

   public UUIDMessage(UUIDMessage other)
   {
      this();
      set(other);
   }

   public void set(UUIDMessage other)
   {
      least_significant_bits_ = other.least_significant_bits_;

      most_significant_bits_ = other.most_significant_bits_;

   }

   public void setLeastSignificantBits(long least_significant_bits)
   {
      least_significant_bits_ = least_significant_bits;
   }
   public long getLeastSignificantBits()
   {
      return least_significant_bits_;
   }

   public void setMostSignificantBits(long most_significant_bits)
   {
      most_significant_bits_ = most_significant_bits;
   }
   public long getMostSignificantBits()
   {
      return most_significant_bits_;
   }


   public static Supplier<UUIDMessagePubSubType> getPubSubType()
   {
      return UUIDMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UUIDMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UUIDMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.least_significant_bits_, other.least_significant_bits_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.most_significant_bits_, other.most_significant_bits_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UUIDMessage)) return false;

      UUIDMessage otherMyClass = (UUIDMessage) other;

      if(this.least_significant_bits_ != otherMyClass.least_significant_bits_) return false;

      if(this.most_significant_bits_ != otherMyClass.most_significant_bits_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UUIDMessage {");
      builder.append("least_significant_bits=");
      builder.append(this.least_significant_bits_);      builder.append(", ");
      builder.append("most_significant_bits=");
      builder.append(this.most_significant_bits_);
      builder.append("}");
      return builder.toString();
   }
}
