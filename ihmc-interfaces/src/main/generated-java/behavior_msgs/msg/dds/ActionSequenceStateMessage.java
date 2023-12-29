package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionSequenceStateMessage extends Packet<ActionSequenceStateMessage> implements Settable<ActionSequenceStateMessage>, EpsilonComparable<ActionSequenceStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ActionSequenceDefinitionMessage definition_;
   /**
            * If the sequence is currently set to proceed automatically
            */
   public boolean automatic_execution_;
   /**
            * The index of the action that is set to execute next
            */
   public int execution_next_index_;
   /**
            * Next action rejection tooltip
            */
   public java.lang.StringBuilder next_action_rejection_tooltip_;
   /**
            * Request manual execution of the next action
            */
   public boolean manual_execution_requested_;
   /**
            * Request action sequence inversion
            */
   public boolean invert_action_sequence_;

   public ActionSequenceStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ActionSequenceDefinitionMessage();
      next_action_rejection_tooltip_ = new java.lang.StringBuilder(255);
   }

   public ActionSequenceStateMessage(ActionSequenceStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionSequenceStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      automatic_execution_ = other.automatic_execution_;

      execution_next_index_ = other.execution_next_index_;

      next_action_rejection_tooltip_.setLength(0);
      next_action_rejection_tooltip_.append(other.next_action_rejection_tooltip_);

      manual_execution_requested_ = other.manual_execution_requested_;

      invert_action_sequence_ = other.invert_action_sequence_;

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
   public behavior_msgs.msg.dds.ActionSequenceDefinitionMessage getDefinition()
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
            * Next action rejection tooltip
            */
   public void setNextActionRejectionTooltip(java.lang.String next_action_rejection_tooltip)
   {
      next_action_rejection_tooltip_.setLength(0);
      next_action_rejection_tooltip_.append(next_action_rejection_tooltip);
   }

   /**
            * Next action rejection tooltip
            */
   public java.lang.String getNextActionRejectionTooltipAsString()
   {
      return getNextActionRejectionTooltip().toString();
   }
   /**
            * Next action rejection tooltip
            */
   public java.lang.StringBuilder getNextActionRejectionTooltip()
   {
      return next_action_rejection_tooltip_;
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
            * Request action sequence inversion
            */
   public void setInvertActionSequence(boolean invert_action_sequence)
   {
      invert_action_sequence_ = invert_action_sequence;
   }
   /**
            * Request action sequence inversion
            */
   public boolean getInvertActionSequence()
   {
      return invert_action_sequence_;
   }


   public static Supplier<ActionSequenceStateMessagePubSubType> getPubSubType()
   {
      return ActionSequenceStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionSequenceStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionSequenceStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.automatic_execution_, other.automatic_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_next_index_, other.execution_next_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.next_action_rejection_tooltip_, other.next_action_rejection_tooltip_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.manual_execution_requested_, other.manual_execution_requested_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.invert_action_sequence_, other.invert_action_sequence_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionSequenceStateMessage)) return false;

      ActionSequenceStateMessage otherMyClass = (ActionSequenceStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.automatic_execution_ != otherMyClass.automatic_execution_) return false;

      if(this.execution_next_index_ != otherMyClass.execution_next_index_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.next_action_rejection_tooltip_, otherMyClass.next_action_rejection_tooltip_)) return false;

      if(this.manual_execution_requested_ != otherMyClass.manual_execution_requested_) return false;

      if(this.invert_action_sequence_ != otherMyClass.invert_action_sequence_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionSequenceStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("automatic_execution=");
      builder.append(this.automatic_execution_);      builder.append(", ");
      builder.append("execution_next_index=");
      builder.append(this.execution_next_index_);      builder.append(", ");
      builder.append("next_action_rejection_tooltip=");
      builder.append(this.next_action_rejection_tooltip_);      builder.append(", ");
      builder.append("manual_execution_requested=");
      builder.append(this.manual_execution_requested_);      builder.append(", ");
      builder.append("invert_action_sequence=");
      builder.append(this.invert_action_sequence_);
      builder.append("}");
      return builder.toString();
   }
}
