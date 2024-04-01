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
            * The ID of the node that generated the message
            */
   public int source_node_id_;
   /**
            * The log message
            */
   public java.lang.StringBuilder message_;

   public BehaviorTreeLogMessage()
   {
      instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      message_ = new java.lang.StringBuilder(255);
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

      source_node_id_ = other.source_node_id_;

      message_.setLength(0);
      message_.append(other.message_);

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
            * The ID of the node that generated the message
            */
   public void setSourceNodeId(int source_node_id)
   {
      source_node_id_ = source_node_id;
   }
   /**
            * The ID of the node that generated the message
            */
   public int getSourceNodeId()
   {
      return source_node_id_;
   }

   /**
            * The log message
            */
   public void setMessage(java.lang.String message)
   {
      message_.setLength(0);
      message_.append(message);
   }

   /**
            * The log message
            */
   public java.lang.String getMessageAsString()
   {
      return getMessage().toString();
   }
   /**
            * The log message
            */
   public java.lang.StringBuilder getMessage()
   {
      return message_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.source_node_id_, other.source_node_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.message_, other.message_, epsilon)) return false;


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

      if(this.source_node_id_ != otherMyClass.source_node_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.message_, otherMyClass.message_)) return false;


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
      builder.append("source_node_id=");
      builder.append(this.source_node_id_);      builder.append(", ");
      builder.append("message=");
      builder.append(this.message_);
      builder.append("}");
      return builder.toString();
   }
}
