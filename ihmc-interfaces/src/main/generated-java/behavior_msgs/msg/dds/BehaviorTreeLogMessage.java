package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeLogMessage extends Packet<BehaviorTreeLogMessage> implements Settable<BehaviorTreeLogMessage>, EpsilonComparable<BehaviorTreeLogMessage>
{
   /**
            * The instant the message was generated. Used also for equals comparison
            */
   public ihmc_common_msgs.msg.dds.InstantMessage instant_;
   /**
            * Log level - Debug(500), Info(400), Warn(300), or Error(200)
            */
   public int log_level_;
   /**
            * The log message, as multiple ASCII bytes because of our 255 string length constraint
            */
   public us.ihmc.idl.IDLSequence.Byte  log_message_;

   public BehaviorTreeLogMessage()
   {
      instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      log_message_ = new us.ihmc.idl.IDLSequence.Byte (2048, "type_9");

   }

   public BehaviorTreeLogMessage(BehaviorTreeLogMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeLogMessage other)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.instant_, instant_);
      log_level_ = other.log_level_;

      log_message_.set(other.log_message_);
   }


   /**
            * The instant the message was generated. Used also for equals comparison
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getInstant()
   {
      return instant_;
   }

   /**
            * Log level - Debug(500), Info(400), Warn(300), or Error(200)
            */
   public void setLogLevel(int log_level)
   {
      log_level_ = log_level;
   }
   /**
            * Log level - Debug(500), Info(400), Warn(300), or Error(200)
            */
   public int getLogLevel()
   {
      return log_level_;
   }


   /**
            * The log message, as multiple ASCII bytes because of our 255 string length constraint
            */
   public us.ihmc.idl.IDLSequence.Byte  getLogMessage()
   {
      return log_message_;
   }


   public static Supplier<BehaviorTreeLogMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeLogMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeLogMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeLogMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.instant_.epsilonEquals(other.instant_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.log_level_, other.log_level_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.log_message_, other.log_message_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeLogMessage)) return false;

      BehaviorTreeLogMessage otherMyClass = (BehaviorTreeLogMessage) other;

      if (!this.instant_.equals(otherMyClass.instant_)) return false;
      if(this.log_level_ != otherMyClass.log_level_) return false;

      if (!this.log_message_.equals(otherMyClass.log_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeLogMessage {");
      builder.append("instant=");
      builder.append(this.instant_);      builder.append(", ");
      builder.append("log_level=");
      builder.append(this.log_level_);      builder.append(", ");
      builder.append("log_message=");
      builder.append(this.log_message_);
      builder.append("}");
      return builder.toString();
   }
}
