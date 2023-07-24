package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StatusLogMessage extends Packet<StatusLogMessage> implements Settable<StatusLogMessage>, EpsilonComparable<StatusLogMessage>
{
   /**
            * Log level
            */
   public int log_level_;
   /**
            * Log message
            */
   public us.ihmc.idl.IDLSequence.Byte  log_message_;

   public StatusLogMessage()
   {
      log_message_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

   }

   public StatusLogMessage(StatusLogMessage other)
   {
      this();
      set(other);
   }

   public void set(StatusLogMessage other)
   {
      log_level_ = other.log_level_;

      log_message_.set(other.log_message_);
   }

   /**
            * Log level
            */
   public void setLogLevel(int log_level)
   {
      log_level_ = log_level;
   }
   /**
            * Log level
            */
   public int getLogLevel()
   {
      return log_level_;
   }


   /**
            * Log message
            */
   public us.ihmc.idl.IDLSequence.Byte  getLogMessage()
   {
      return log_message_;
   }


   public static Supplier<StatusLogMessagePubSubType> getPubSubType()
   {
      return StatusLogMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StatusLogMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StatusLogMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.log_level_, other.log_level_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.log_message_, other.log_message_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StatusLogMessage)) return false;

      StatusLogMessage otherMyClass = (StatusLogMessage) other;

      if(this.log_level_ != otherMyClass.log_level_) return false;

      if (!this.log_message_.equals(otherMyClass.log_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StatusLogMessage {");
      builder.append("log_level=");
      builder.append(this.log_level_);      builder.append(", ");
      builder.append("log_message=");
      builder.append(this.log_message_);
      builder.append("}");
      return builder.toString();
   }
}
