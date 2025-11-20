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
            * The positional vector from frame B to frame A in world frame
            */
   public us.ihmc.euclid.tuple3D.Vector3D b_to_a_;
   /**
            * Whether frame A is present
            */
   public boolean frame_a_is_present_;
   /**
            * Whether frame B is present
            */
   public boolean frame_b_is_present_;

   public ConditionNodeStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.LeafNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ConditionNodeDefinitionMessage();
      b_to_a_ = new us.ihmc.euclid.tuple3D.Vector3D();
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

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.b_to_a_, b_to_a_);
      frame_a_is_present_ = other.frame_a_is_present_;

      frame_b_is_present_ = other.frame_b_is_present_;

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
            * The positional vector from frame B to frame A in world frame
            */
   public us.ihmc.euclid.tuple3D.Vector3D getBToA()
   {
      return b_to_a_;
   }

   /**
            * Whether frame A is present
            */
   public void setFrameAIsPresent(boolean frame_a_is_present)
   {
      frame_a_is_present_ = frame_a_is_present;
   }
   /**
            * Whether frame A is present
            */
   public boolean getFrameAIsPresent()
   {
      return frame_a_is_present_;
   }

   /**
            * Whether frame B is present
            */
   public void setFrameBIsPresent(boolean frame_b_is_present)
   {
      frame_b_is_present_ = frame_b_is_present;
   }
   /**
            * Whether frame B is present
            */
   public boolean getFrameBIsPresent()
   {
      return frame_b_is_present_;
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

      if (!this.b_to_a_.epsilonEquals(other.b_to_a_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.frame_a_is_present_, other.frame_a_is_present_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.frame_b_is_present_, other.frame_b_is_present_, epsilon)) return false;


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

      if (!this.b_to_a_.equals(otherMyClass.b_to_a_)) return false;
      if(this.frame_a_is_present_ != otherMyClass.frame_a_is_present_) return false;

      if(this.frame_b_is_present_ != otherMyClass.frame_b_is_present_) return false;


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
      builder.append("b_to_a=");
      builder.append(this.b_to_a_);      builder.append(", ");
      builder.append("frame_a_is_present=");
      builder.append(this.frame_a_is_present_);      builder.append(", ");
      builder.append("frame_b_is_present=");
      builder.append(this.frame_b_is_present_);
      builder.append("}");
      return builder.toString();
   }
}
