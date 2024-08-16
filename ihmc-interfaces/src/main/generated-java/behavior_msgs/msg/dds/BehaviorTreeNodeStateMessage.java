package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeNodeStateMessage extends Packet<BehaviorTreeNodeStateMessage> implements Settable<BehaviorTreeNodeStateMessage>, EpsilonComparable<BehaviorTreeNodeStateMessage>
{
   /**
            * The node's unique ID
            */
   public long id_;
   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public boolean is_active_;
   /**
            * A mechanism for confirming and ending a freeze early
            */
   public ihmc_common_msgs.msg.dds.ConfirmableRequestMessage confirmable_request_;
   /**
            * Recent log messages
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeLogMessage>  recent_log_messages_;

   public BehaviorTreeNodeStateMessage()
   {
      confirmable_request_ = new ihmc_common_msgs.msg.dds.ConfirmableRequestMessage();
      recent_log_messages_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeLogMessage> (50, new behavior_msgs.msg.dds.BehaviorTreeLogMessagePubSubType());

   }

   public BehaviorTreeNodeStateMessage(BehaviorTreeNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeStateMessage other)
   {
      id_ = other.id_;

      is_active_ = other.is_active_;

      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.staticCopy(other.confirmable_request_, confirmable_request_);
      recent_log_messages_.set(other.recent_log_messages_);
   }

   /**
            * The node's unique ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The node's unique ID
            */
   public long getId()
   {
      return id_;
   }

   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public void setIsActive(boolean is_active)
   {
      is_active_ = is_active;
   }
   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public boolean getIsActive()
   {
      return is_active_;
   }


   /**
            * A mechanism for confirming and ending a freeze early
            */
   public ihmc_common_msgs.msg.dds.ConfirmableRequestMessage getConfirmableRequest()
   {
      return confirmable_request_;
   }


   /**
            * Recent log messages
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeLogMessage>  getRecentLogMessages()
   {
      return recent_log_messages_;
   }


   public static Supplier<BehaviorTreeNodeStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_active_, other.is_active_, epsilon)) return false;

      if (!this.confirmable_request_.epsilonEquals(other.confirmable_request_, epsilon)) return false;
      if (this.recent_log_messages_.size() != other.recent_log_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.recent_log_messages_.size(); i++)
         {  if (!this.recent_log_messages_.get(i).epsilonEquals(other.recent_log_messages_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeNodeStateMessage)) return false;

      BehaviorTreeNodeStateMessage otherMyClass = (BehaviorTreeNodeStateMessage) other;

      if(this.id_ != otherMyClass.id_) return false;

      if(this.is_active_ != otherMyClass.is_active_) return false;

      if (!this.confirmable_request_.equals(otherMyClass.confirmable_request_)) return false;
      if (!this.recent_log_messages_.equals(otherMyClass.recent_log_messages_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeStateMessage {");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("is_active=");
      builder.append(this.is_active_);      builder.append(", ");
      builder.append("confirmable_request=");
      builder.append(this.confirmable_request_);      builder.append(", ");
      builder.append("recent_log_messages=");
      builder.append(this.recent_log_messages_);
      builder.append("}");
      return builder.toString();
   }
}
