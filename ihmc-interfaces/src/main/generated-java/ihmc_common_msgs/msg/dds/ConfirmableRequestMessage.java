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
            * If this message is a request to change something.
            */
   public boolean is_request_;
   /**
            * If this message is a confirmation that the request has been received by the other side.
            */
   public boolean is_confirmation_;
   /**
            * Request number, monotonically increasing
            */
   public long request_number_;
   /**
            * Confirmation number, the value of the last received request number
            */
   public long confirmation_number_;

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
      is_request_ = other.is_request_;

      is_confirmation_ = other.is_confirmation_;

      request_number_ = other.request_number_;

      confirmation_number_ = other.confirmation_number_;

   }

   /**
            * If this message is a request to change something.
            */
   public void setIsRequest(boolean is_request)
   {
      is_request_ = is_request;
   }
   /**
            * If this message is a request to change something.
            */
   public boolean getIsRequest()
   {
      return is_request_;
   }

   /**
            * If this message is a confirmation that the request has been received by the other side.
            */
   public void setIsConfirmation(boolean is_confirmation)
   {
      is_confirmation_ = is_confirmation;
   }
   /**
            * If this message is a confirmation that the request has been received by the other side.
            */
   public boolean getIsConfirmation()
   {
      return is_confirmation_;
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

   /**
            * Confirmation number, the value of the last received request number
            */
   public void setConfirmationNumber(long confirmation_number)
   {
      confirmation_number_ = confirmation_number;
   }
   /**
            * Confirmation number, the value of the last received request number
            */
   public long getConfirmationNumber()
   {
      return confirmation_number_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_request_, other.is_request_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_confirmation_, other.is_confirmation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.request_number_, other.request_number_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confirmation_number_, other.confirmation_number_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConfirmableRequestMessage)) return false;

      ConfirmableRequestMessage otherMyClass = (ConfirmableRequestMessage) other;

      if(this.is_request_ != otherMyClass.is_request_) return false;

      if(this.is_confirmation_ != otherMyClass.is_confirmation_) return false;

      if(this.request_number_ != otherMyClass.request_number_) return false;

      if(this.confirmation_number_ != otherMyClass.confirmation_number_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConfirmableRequestMessage {");
      builder.append("is_request=");
      builder.append(this.is_request_);      builder.append(", ");
      builder.append("is_confirmation=");
      builder.append(this.is_confirmation_);      builder.append(", ");
      builder.append("request_number=");
      builder.append(this.request_number_);      builder.append(", ");
      builder.append("confirmation_number=");
      builder.append(this.confirmation_number_);
      builder.append("}");
      return builder.toString();
   }
}
