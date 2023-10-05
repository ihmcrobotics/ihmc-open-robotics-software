package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorActionDefinitionMessage extends Packet<BehaviorActionDefinitionMessage> implements Settable<BehaviorActionDefinitionMessage>, EpsilonComparable<BehaviorActionDefinitionMessage>
{
   /**
            * Human readable description of what the action does
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  description_;
   /**
            * Execute with next action
            */
   public boolean execute_with_next_action_;

   public BehaviorActionDefinitionMessage()
   {
      description_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (1, "type_d");
   }

   public BehaviorActionDefinitionMessage(BehaviorActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorActionDefinitionMessage other)
   {
      description_.set(other.description_);
      execute_with_next_action_ = other.execute_with_next_action_;

   }


   /**
            * Human readable description of what the action does
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getDescription()
   {
      return description_;
   }

   /**
            * Execute with next action
            */
   public void setExecuteWithNextAction(boolean execute_with_next_action)
   {
      execute_with_next_action_ = execute_with_next_action;
   }
   /**
            * Execute with next action
            */
   public boolean getExecuteWithNextAction()
   {
      return execute_with_next_action_;
   }


   public static Supplier<BehaviorActionDefinitionMessagePubSubType> getPubSubType()
   {
      return BehaviorActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.description_, other.description_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_with_next_action_, other.execute_with_next_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorActionDefinitionMessage)) return false;

      BehaviorActionDefinitionMessage otherMyClass = (BehaviorActionDefinitionMessage) other;

      if (!this.description_.equals(otherMyClass.description_)) return false;
      if(this.execute_with_next_action_ != otherMyClass.execute_with_next_action_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorActionDefinitionMessage {");
      builder.append("description=");
      builder.append(this.description_);      builder.append(", ");
      builder.append("execute_with_next_action=");
      builder.append(this.execute_with_next_action_);
      builder.append("}");
      return builder.toString();
   }
}
