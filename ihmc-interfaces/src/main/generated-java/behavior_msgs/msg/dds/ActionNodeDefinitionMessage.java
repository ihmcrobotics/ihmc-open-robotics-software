package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionNodeDefinitionMessage extends Packet<ActionNodeDefinitionMessage> implements Settable<ActionNodeDefinitionMessage>, EpsilonComparable<ActionNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * Execute with next action
            */
   public java.lang.StringBuilder execute_after_action_;

   public ActionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
      execute_after_action_ = new java.lang.StringBuilder(255);
   }

   public ActionNodeDefinitionMessage(ActionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      execute_after_action_.setLength(0);
      execute_after_action_.append(other.execute_after_action_);

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Execute with next action
            */
   public void setExecuteAfterAction(java.lang.String execute_after_action)
   {
      execute_after_action_.setLength(0);
      execute_after_action_.append(execute_after_action);
   }

   /**
            * Execute with next action
            */
   public java.lang.String getExecuteAfterActionAsString()
   {
      return getExecuteAfterAction().toString();
   }
   /**
            * Execute with next action
            */
   public java.lang.StringBuilder getExecuteAfterAction()
   {
      return execute_after_action_;
   }


   public static Supplier<ActionNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return ActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.execute_after_action_, other.execute_after_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionNodeDefinitionMessage)) return false;

      ActionNodeDefinitionMessage otherMyClass = (ActionNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.execute_after_action_, otherMyClass.execute_after_action_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("execute_after_action=");
      builder.append(this.execute_after_action_);
      builder.append("}");
      return builder.toString();
   }
}
