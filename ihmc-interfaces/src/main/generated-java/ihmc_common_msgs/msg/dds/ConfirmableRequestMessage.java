package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A way of avoiding local data get overriden before
       * it is received by peers. A message can be a request
       * and confirmation at the same time.
       */
public class ConfirmableRequestMessage extends Packet<ConfirmableRequestMessage> implements Settable<ConfirmableRequestMessage>, EpsilonComparable<ConfirmableRequestMessage>
{
   /**
            * Recent request numbers, monotonically increasing
            * The size of this is roughly the max amount of expected dropped messages in 1 second
            */
   public us.ihmc.idl.IDLSequence.Long  request_numbers_;
   /**
            * Recent confirmation numbers, the values of received request numbers
            * The size of this is roughly the max amount of expected dropped messages in 1 second
            */
   public us.ihmc.idl.IDLSequence.Long  confirmation_numbers_;

   public ConfirmableRequestMessage()
   {
      request_numbers_ = new us.ihmc.idl.IDLSequence.Long (10, "type_4");

      confirmation_numbers_ = new us.ihmc.idl.IDLSequence.Long (10, "type_4");

   }

   public ConfirmableRequestMessage(ConfirmableRequestMessage other)
   {
      this();
      set(other);
   }

   public void set(ConfirmableRequestMessage other)
   {
      request_numbers_.set(other.request_numbers_);
      confirmation_numbers_.set(other.confirmation_numbers_);
   }


   /**
            * Recent request numbers, monotonically increasing
            * The size of this is roughly the max amount of expected dropped messages in 1 second
            */
   public us.ihmc.idl.IDLSequence.Long  getRequestNumbers()
   {
      return request_numbers_;
   }


   /**
            * Recent confirmation numbers, the values of received request numbers
            * The size of this is roughly the max amount of expected dropped messages in 1 second
            */
   public us.ihmc.idl.IDLSequence.Long  getConfirmationNumbers()
   {
      return confirmation_numbers_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.request_numbers_, other.request_numbers_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.confirmation_numbers_, other.confirmation_numbers_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConfirmableRequestMessage)) return false;

      ConfirmableRequestMessage otherMyClass = (ConfirmableRequestMessage) other;

      if (!this.request_numbers_.equals(otherMyClass.request_numbers_)) return false;
      if (!this.confirmation_numbers_.equals(otherMyClass.confirmation_numbers_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConfirmableRequestMessage {");
      builder.append("request_numbers=");
      builder.append(this.request_numbers_);      builder.append(", ");
      builder.append("confirmation_numbers=");
      builder.append(this.confirmation_numbers_);
      builder.append("}");
      return builder.toString();
   }
}
