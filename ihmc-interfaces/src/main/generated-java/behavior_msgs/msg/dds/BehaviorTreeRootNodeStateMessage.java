package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeRootNodeStateMessage extends Packet<BehaviorTreeRootNodeStateMessage> implements Settable<BehaviorTreeRootNodeStateMessage>, EpsilonComparable<BehaviorTreeRootNodeStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessage definition_;
   /**
            * If the sequence is currently set to proceed automatically
            */
   public boolean automatic_execution_;
   /**
            * The index of the action that is set to execute next
            */
   public int execution_next_index_;
   /**
            * Request manual execution of the next action
            */
   public boolean manual_execution_requested_;
   /**
            * Allows the operator to disable concurrency during authoring
            */
   public boolean concurrency_enabled_;

   public BehaviorTreeRootNodeStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessage();
   }

   public BehaviorTreeRootNodeStateMessage(BehaviorTreeRootNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeRootNodeStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      automatic_execution_ = other.automatic_execution_;

      execution_next_index_ = other.execution_next_index_;

      manual_execution_requested_ = other.manual_execution_requested_;

      concurrency_enabled_ = other.concurrency_enabled_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * If the sequence is currently set to proceed automatically
            */
   public void setAutomaticExecution(boolean automatic_execution)
   {
      automatic_execution_ = automatic_execution;
   }
   /**
            * If the sequence is currently set to proceed automatically
            */
   public boolean getAutomaticExecution()
   {
      return automatic_execution_;
   }

   /**
            * The index of the action that is set to execute next
            */
   public void setExecutionNextIndex(int execution_next_index)
   {
      execution_next_index_ = execution_next_index;
   }
   /**
            * The index of the action that is set to execute next
            */
   public int getExecutionNextIndex()
   {
      return execution_next_index_;
   }

   /**
            * Request manual execution of the next action
            */
   public void setManualExecutionRequested(boolean manual_execution_requested)
   {
      manual_execution_requested_ = manual_execution_requested;
   }
   /**
            * Request manual execution of the next action
            */
   public boolean getManualExecutionRequested()
   {
      return manual_execution_requested_;
   }

   /**
            * Allows the operator to disable concurrency during authoring
            */
   public void setConcurrencyEnabled(boolean concurrency_enabled)
   {
      concurrency_enabled_ = concurrency_enabled;
   }
   /**
            * Allows the operator to disable concurrency during authoring
            */
   public boolean getConcurrencyEnabled()
   {
      return concurrency_enabled_;
   }


   public static Supplier<BehaviorTreeRootNodeStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeRootNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeRootNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeRootNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.automatic_execution_, other.automatic_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_next_index_, other.execution_next_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.manual_execution_requested_, other.manual_execution_requested_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.concurrency_enabled_, other.concurrency_enabled_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeRootNodeStateMessage)) return false;

      BehaviorTreeRootNodeStateMessage otherMyClass = (BehaviorTreeRootNodeStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.automatic_execution_ != otherMyClass.automatic_execution_) return false;

      if(this.execution_next_index_ != otherMyClass.execution_next_index_) return false;

      if(this.manual_execution_requested_ != otherMyClass.manual_execution_requested_) return false;

      if(this.concurrency_enabled_ != otherMyClass.concurrency_enabled_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeRootNodeStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("automatic_execution=");
      builder.append(this.automatic_execution_);      builder.append(", ");
      builder.append("execution_next_index=");
      builder.append(this.execution_next_index_);      builder.append(", ");
      builder.append("manual_execution_requested=");
      builder.append(this.manual_execution_requested_);      builder.append(", ");
      builder.append("concurrency_enabled=");
      builder.append(this.concurrency_enabled_);
      builder.append("}");
      return builder.toString();
   }
}
