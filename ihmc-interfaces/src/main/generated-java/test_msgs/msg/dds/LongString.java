package test_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LongString extends Packet<LongString> implements Settable<LongString>, EpsilonComparable<LongString>
{
   /**
            * String as multiple ASCII bytes because of our 255 string length constraint
            */
   public us.ihmc.idl.IDLSequence.Byte  long_string_;

   public LongString()
   {
      long_string_ = new us.ihmc.idl.IDLSequence.Byte (2048, "type_9");

   }

   public LongString(LongString other)
   {
      this();
      set(other);
   }

   public void set(LongString other)
   {
      long_string_.set(other.long_string_);
   }


   /**
            * String as multiple ASCII bytes because of our 255 string length constraint
            */
   public us.ihmc.idl.IDLSequence.Byte  getLongString()
   {
      return long_string_;
   }


   public static Supplier<LongStringPubSubType> getPubSubType()
   {
      return LongStringPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LongStringPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LongString other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.long_string_, other.long_string_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LongString)) return false;

      LongString otherMyClass = (LongString) other;

      if (!this.long_string_.equals(otherMyClass.long_string_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LongString {");
      builder.append("long_string=");
      builder.append(this.long_string_);
      builder.append("}");
      return builder.toString();
   }
}
