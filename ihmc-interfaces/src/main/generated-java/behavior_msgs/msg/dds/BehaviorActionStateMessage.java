package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorActionStateMessage extends Packet<BehaviorActionStateMessage> implements Settable<BehaviorActionStateMessage>, EpsilonComparable<BehaviorActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage node_state_;
   /**
            * The action's unique ID
            */
   public long id_;
   /**
            * The action's index in the sequence
            */
   public int action_index_;
   /**
            * If the action is next for execution
            */
   public boolean is_next_for_execution_;
   /**
            * If the action is to be executed concurrently
            */
   public boolean is_to_be_executed_concurrently_;
   /**
            * If the node is able to execution
            */
   public boolean can_execute_;
   /**
            * If the node is currently executing
            */
   public boolean is_executing_;

   public BehaviorActionStateMessage()
   {
      node_state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
   }

   public BehaviorActionStateMessage(BehaviorActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorActionStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.node_state_, node_state_);
      id_ = other.id_;

      action_index_ = other.action_index_;

      is_next_for_execution_ = other.is_next_for_execution_;

      is_to_be_executed_concurrently_ = other.is_to_be_executed_concurrently_;

      can_execute_ = other.can_execute_;

      is_executing_ = other.is_executing_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getNodeState()
   {
      return node_state_;
   }

   /**
            * The action's unique ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The action's unique ID
            */
   public long getId()
   {
      return id_;
   }

   /**
            * The action's index in the sequence
            */
   public void setActionIndex(int action_index)
   {
      action_index_ = action_index;
   }
   /**
            * The action's index in the sequence
            */
   public int getActionIndex()
   {
      return action_index_;
   }

   /**
            * If the action is next for execution
            */
   public void setIsNextForExecution(boolean is_next_for_execution)
   {
      is_next_for_execution_ = is_next_for_execution;
   }
   /**
            * If the action is next for execution
            */
   public boolean getIsNextForExecution()
   {
      return is_next_for_execution_;
   }

   /**
            * If the action is to be executed concurrently
            */
   public void setIsToBeExecutedConcurrently(boolean is_to_be_executed_concurrently)
   {
      is_to_be_executed_concurrently_ = is_to_be_executed_concurrently;
   }
   /**
            * If the action is to be executed concurrently
            */
   public boolean getIsToBeExecutedConcurrently()
   {
      return is_to_be_executed_concurrently_;
   }

   /**
            * If the node is able to execution
            */
   public void setCanExecute(boolean can_execute)
   {
      can_execute_ = can_execute;
   }
   /**
            * If the node is able to execution
            */
   public boolean getCanExecute()
   {
      return can_execute_;
   }

   /**
            * If the node is currently executing
            */
   public void setIsExecuting(boolean is_executing)
   {
      is_executing_ = is_executing;
   }
   /**
            * If the node is currently executing
            */
   public boolean getIsExecuting()
   {
      return is_executing_;
   }


   public static Supplier<BehaviorActionStateMessagePubSubType> getPubSubType()
   {
      return BehaviorActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.node_state_.epsilonEquals(other.node_state_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.action_index_, other.action_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_next_for_execution_, other.is_next_for_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_to_be_executed_concurrently_, other.is_to_be_executed_concurrently_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.can_execute_, other.can_execute_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_executing_, other.is_executing_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorActionStateMessage)) return false;

      BehaviorActionStateMessage otherMyClass = (BehaviorActionStateMessage) other;

      if (!this.node_state_.equals(otherMyClass.node_state_)) return false;
      if(this.id_ != otherMyClass.id_) return false;

      if(this.action_index_ != otherMyClass.action_index_) return false;

      if(this.is_next_for_execution_ != otherMyClass.is_next_for_execution_) return false;

      if(this.is_to_be_executed_concurrently_ != otherMyClass.is_to_be_executed_concurrently_) return false;

      if(this.can_execute_ != otherMyClass.can_execute_) return false;

      if(this.is_executing_ != otherMyClass.is_executing_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorActionStateMessage {");
      builder.append("node_state=");
      builder.append(this.node_state_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("action_index=");
      builder.append(this.action_index_);      builder.append(", ");
      builder.append("is_next_for_execution=");
      builder.append(this.is_next_for_execution_);      builder.append(", ");
      builder.append("is_to_be_executed_concurrently=");
      builder.append(this.is_to_be_executed_concurrently_);      builder.append(", ");
      builder.append("can_execute=");
      builder.append(this.can_execute_);      builder.append(", ");
      builder.append("is_executing=");
      builder.append(this.is_executing_);
      builder.append("}");
      return builder.toString();
   }
}
