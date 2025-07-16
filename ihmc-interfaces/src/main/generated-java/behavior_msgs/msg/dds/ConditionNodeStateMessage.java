package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * COUNTER TYPE
       * LLM TYPE
       * PROXIMITY TYPE
       */
public class ConditionNodeStateMessage extends Packet<ConditionNodeStateMessage> implements Settable<ConditionNodeStateMessage>, EpsilonComparable<ConditionNodeStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.LeafNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ConditionNodeDefinitionMessage definition_;
   /**
            * The current count of the counter
            */
   public long count_;
   /**
            * A request to reset the LLM's context now
            */
   public boolean request_reset_context_;
   /**
            * The current distance of the object from the reference frame
            */
   public double current_distance_;
   /**
            * Whether frame to evaluate the proximity is missing
            */
   public boolean missing_frame_;

   public ConditionNodeStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.LeafNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ConditionNodeDefinitionMessage();
   }

   public ConditionNodeStateMessage(ConditionNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ConditionNodeStateMessage other)
   {
      behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      count_ = other.count_;

      request_reset_context_ = other.request_reset_context_;

      current_distance_ = other.current_distance_;

      missing_frame_ = other.missing_frame_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.LeafNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ConditionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The current count of the counter
            */
   public void setCount(long count)
   {
      count_ = count;
   }
   /**
            * The current count of the counter
            */
   public long getCount()
   {
      return count_;
   }

   /**
            * A request to reset the LLM's context now
            */
   public void setRequestResetContext(boolean request_reset_context)
   {
      request_reset_context_ = request_reset_context;
   }
   /**
            * A request to reset the LLM's context now
            */
   public boolean getRequestResetContext()
   {
      return request_reset_context_;
   }

   /**
            * The current distance of the object from the reference frame
            */
   public void setCurrentDistance(double current_distance)
   {
      current_distance_ = current_distance;
   }
   /**
            * The current distance of the object from the reference frame
            */
   public double getCurrentDistance()
   {
      return current_distance_;
   }

   /**
            * Whether frame to evaluate the proximity is missing
            */
   public void setMissingFrame(boolean missing_frame)
   {
      missing_frame_ = missing_frame;
   }
   /**
            * Whether frame to evaluate the proximity is missing
            */
   public boolean getMissingFrame()
   {
      return missing_frame_;
   }


   public static Supplier<ConditionNodeStateMessagePubSubType> getPubSubType()
   {
      return ConditionNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConditionNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConditionNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.count_, other.count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_reset_context_, other.request_reset_context_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_distance_, other.current_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.missing_frame_, other.missing_frame_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConditionNodeStateMessage)) return false;

      ConditionNodeStateMessage otherMyClass = (ConditionNodeStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.count_ != otherMyClass.count_) return false;

      if(this.request_reset_context_ != otherMyClass.request_reset_context_) return false;

      if(this.current_distance_ != otherMyClass.current_distance_) return false;

      if(this.missing_frame_ != otherMyClass.missing_frame_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConditionNodeStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("count=");
      builder.append(this.count_);      builder.append(", ");
      builder.append("request_reset_context=");
      builder.append(this.request_reset_context_);      builder.append(", ");
      builder.append("current_distance=");
      builder.append(this.current_distance_);      builder.append(", ");
      builder.append("missing_frame=");
      builder.append(this.missing_frame_);
      builder.append("}");
      return builder.toString();
   }
}
