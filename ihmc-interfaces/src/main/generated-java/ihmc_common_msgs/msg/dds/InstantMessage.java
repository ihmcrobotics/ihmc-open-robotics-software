package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * An instantaneous point on the time-line.
       * Corresponds to java.time.Instant. Read about it:
       * https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/time/Instant.html
       */
public class InstantMessage extends Packet<InstantMessage> implements Settable<InstantMessage>, EpsilonComparable<InstantMessage>
{
   /**
            * Seconds elapsed since the standard Java epoch of 1970-01-01T00:00:00Z
            */
   public long seconds_since_epoch_;
   /**
            * Additional nanoseconds which will always be between 0 and 999,999,999
            */
   public long additional_nanos_;

   public InstantMessage()
   {
   }

   public InstantMessage(InstantMessage other)
   {
      this();
      set(other);
   }

   public void set(InstantMessage other)
   {
      seconds_since_epoch_ = other.seconds_since_epoch_;

      additional_nanos_ = other.additional_nanos_;

   }

   /**
            * Seconds elapsed since the standard Java epoch of 1970-01-01T00:00:00Z
            */
   public void setSecondsSinceEpoch(long seconds_since_epoch)
   {
      seconds_since_epoch_ = seconds_since_epoch;
   }
   /**
            * Seconds elapsed since the standard Java epoch of 1970-01-01T00:00:00Z
            */
   public long getSecondsSinceEpoch()
   {
      return seconds_since_epoch_;
   }

   /**
            * Additional nanoseconds which will always be between 0 and 999,999,999
            */
   public void setAdditionalNanos(long additional_nanos)
   {
      additional_nanos_ = additional_nanos;
   }
   /**
            * Additional nanoseconds which will always be between 0 and 999,999,999
            */
   public long getAdditionalNanos()
   {
      return additional_nanos_;
   }


   public static Supplier<InstantMessagePubSubType> getPubSubType()
   {
      return InstantMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return InstantMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(InstantMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.seconds_since_epoch_, other.seconds_since_epoch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.additional_nanos_, other.additional_nanos_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof InstantMessage)) return false;

      InstantMessage otherMyClass = (InstantMessage) other;

      if(this.seconds_since_epoch_ != otherMyClass.seconds_since_epoch_) return false;

      if(this.additional_nanos_ != otherMyClass.additional_nanos_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("InstantMessage {");
      builder.append("seconds_since_epoch=");
      builder.append(this.seconds_since_epoch_);      builder.append(", ");
      builder.append("additional_nanos=");
      builder.append(this.additional_nanos_);
      builder.append("}");
      return builder.toString();
   }
}
