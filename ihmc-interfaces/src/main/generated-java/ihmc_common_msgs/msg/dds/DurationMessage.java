package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message for java.time.Duration. See:
       * https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/time/Duration.html
       */
public class DurationMessage extends Packet<DurationMessage> implements Settable<DurationMessage>, EpsilonComparable<DurationMessage>
{
   public long seconds_;
   public int nanos_;

   public DurationMessage()
   {
   }

   public DurationMessage(DurationMessage other)
   {
      this();
      set(other);
   }

   public void set(DurationMessage other)
   {
      seconds_ = other.seconds_;

      nanos_ = other.nanos_;

   }

   public void setSeconds(long seconds)
   {
      seconds_ = seconds;
   }
   public long getSeconds()
   {
      return seconds_;
   }

   public void setNanos(int nanos)
   {
      nanos_ = nanos;
   }
   public int getNanos()
   {
      return nanos_;
   }


   public static Supplier<DurationMessagePubSubType> getPubSubType()
   {
      return DurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.seconds_, other.seconds_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nanos_, other.nanos_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DurationMessage)) return false;

      DurationMessage otherMyClass = (DurationMessage) other;

      if(this.seconds_ != otherMyClass.seconds_) return false;

      if(this.nanos_ != otherMyClass.nanos_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DurationMessage {");
      builder.append("seconds=");
      builder.append(this.seconds_);      builder.append(", ");
      builder.append("nanos=");
      builder.append(this.nanos_);
      builder.append("}");
      return builder.toString();
   }
}
