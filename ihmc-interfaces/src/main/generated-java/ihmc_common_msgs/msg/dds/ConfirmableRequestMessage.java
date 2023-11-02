package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A way of avoiding local data get overriden before
       * it is received by peers.
       */
public class ConfirmableRequestMessage extends Packet<ConfirmableRequestMessage> implements Settable<ConfirmableRequestMessage>, EpsilonComparable<ConfirmableRequestMessage>
{
   /**
          * Nothing special has happened. This should trigger no effects.
          */
   public static final byte NOOP = (byte) 0;
   /**
          * This message is a request to change something.
          */
   public static final byte REQUEST = (byte) 1;
   /**
          * Confirmation that the request has been received by the other side.
          */
   public static final byte CONFIRMATION = (byte) 2;
   /**
            * Holds one of the above constant values.
            */
   public int value_;
   /**
            * Request number, monotonically increasing
            */
   public long request_number_;

   public ConfirmableRequestMessage()
   {
   }

   public ConfirmableRequestMessage(ConfirmableRequestMessage other)
   {
      this();
      set(other);
   }

   public void set(ConfirmableRequestMessage other)
   {
      value_ = other.value_;

      request_number_ = other.request_number_;

   }

   /**
            * Holds one of the above constant values.
            */
   public void setValue(int value)
   {
      value_ = value;
   }
   /**
            * Holds one of the above constant values.
            */
   public int getValue()
   {
      return value_;
   }

   /**
            * Request number, monotonically increasing
            */
   public void setRequestNumber(long request_number)
   {
      request_number_ = request_number;
   }
   /**
            * Request number, monotonically increasing
            */
   public long getRequestNumber()
   {
      return request_number_;
   }


   public static Supplier<ConfirmableRequestMessagePubSubType> getPubSubType()
   {
      return ConfirmableRequestMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConfirmableRequestMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConfirmableRequestMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_, other.value_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.request_number_, other.request_number_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConfirmableRequestMessage)) return false;

      ConfirmableRequestMessage otherMyClass = (ConfirmableRequestMessage) other;

      if(this.value_ != otherMyClass.value_) return false;

      if(this.request_number_ != otherMyClass.request_number_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConfirmableRequestMessage {");
      builder.append("value=");
      builder.append(this.value_);      builder.append(", ");
      builder.append("request_number=");
      builder.append(this.request_number_);
      builder.append("}");
      return builder.toString();
   }
}
