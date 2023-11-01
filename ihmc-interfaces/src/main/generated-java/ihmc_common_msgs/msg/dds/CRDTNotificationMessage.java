package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A way of sending a notification via CRDT synced state messages.
       * By using 3 values, we can set a notification and receive a
       * confirmation that the other side polled it without sync errors.
       */
public class CRDTNotificationMessage extends Packet<CRDTNotificationMessage> implements Settable<CRDTNotificationMessage>, EpsilonComparable<CRDTNotificationMessage>
{
   /**
          * Nothing special has happened. This should have no effect of subscribers.
          */
   public static final byte NOOP = (byte) 0;
   /**
          * A set of the notification on the other side has been requested.
          */
   public static final byte SET_REQUEST = (byte) 1;
   /**
          * Confirmation that the notification has been polled by the other side.
          */
   public static final byte POLL_CONFIRMED = (byte) 2;
   /**
            * Holds one of the above constant values.
            */
   public int value_;

   public CRDTNotificationMessage()
   {
   }

   public CRDTNotificationMessage(CRDTNotificationMessage other)
   {
      this();
      set(other);
   }

   public void set(CRDTNotificationMessage other)
   {
      value_ = other.value_;

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


   public static Supplier<CRDTNotificationMessagePubSubType> getPubSubType()
   {
      return CRDTNotificationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CRDTNotificationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CRDTNotificationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_, other.value_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CRDTNotificationMessage)) return false;

      CRDTNotificationMessage otherMyClass = (CRDTNotificationMessage) other;

      if(this.value_ != otherMyClass.value_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CRDTNotificationMessage {");
      builder.append("value=");
      builder.append(this.value_);
      builder.append("}");
      return builder.toString();
   }
}
