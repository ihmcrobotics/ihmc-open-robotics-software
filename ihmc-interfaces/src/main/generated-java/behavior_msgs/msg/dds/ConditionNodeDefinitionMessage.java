package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * COUNTER TYPE
       * LLM TYPE
       */
public class ConditionNodeDefinitionMessage extends Packet<ConditionNodeDefinitionMessage> implements Settable<ConditionNodeDefinitionMessage>, EpsilonComparable<ConditionNodeDefinitionMessage>
{
   public static final byte COUNTER_TYPE = (byte) 0;
   public static final byte LLM_TYPE = (byte) 1;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage definition_;
   /**
            * The type of condtion as defined above
            */
   public byte type_;
   /**
            * The number of times to fail before passing
            */
   public long count_to_;
   /**
            * Whether to reset the context on each run
            */
   public boolean reset_context_each_run_;
   /**
            * Whether to generate behavior state information and add it to the prompt
            */
   public boolean inject_behavior_state_;
   /**
            * Whether to generate environment state information and add it to the prompt
            */
   public boolean inject_environment_state_;
   /**
            * The prompt given once at the very beginning
            */
   public java.lang.StringBuilder system_;
   /**
            * The prompt defining how to make the boolean decision
            */
   public java.lang.StringBuilder prompt_;

   public ConditionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.LeafNodeDefinitionMessage();
      system_ = new java.lang.StringBuilder(10000);
      prompt_ = new java.lang.StringBuilder(10000);
   }

   public ConditionNodeDefinitionMessage(ConditionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ConditionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      type_ = other.type_;

      count_to_ = other.count_to_;

      reset_context_each_run_ = other.reset_context_each_run_;

      inject_behavior_state_ = other.inject_behavior_state_;

      inject_environment_state_ = other.inject_environment_state_;

      system_.setLength(0);
      system_.append(other.system_);

      prompt_.setLength(0);
      prompt_.append(other.prompt_);

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The type of condtion as defined above
            */
   public void setType(byte type)
   {
      type_ = type;
   }
   /**
            * The type of condtion as defined above
            */
   public byte getType()
   {
      return type_;
   }

   /**
            * The number of times to fail before passing
            */
   public void setCountTo(long count_to)
   {
      count_to_ = count_to;
   }
   /**
            * The number of times to fail before passing
            */
   public long getCountTo()
   {
      return count_to_;
   }

   /**
            * Whether to reset the context on each run
            */
   public void setResetContextEachRun(boolean reset_context_each_run)
   {
      reset_context_each_run_ = reset_context_each_run;
   }
   /**
            * Whether to reset the context on each run
            */
   public boolean getResetContextEachRun()
   {
      return reset_context_each_run_;
   }

   /**
            * Whether to generate behavior state information and add it to the prompt
            */
   public void setInjectBehaviorState(boolean inject_behavior_state)
   {
      inject_behavior_state_ = inject_behavior_state;
   }
   /**
            * Whether to generate behavior state information and add it to the prompt
            */
   public boolean getInjectBehaviorState()
   {
      return inject_behavior_state_;
   }

   /**
            * Whether to generate environment state information and add it to the prompt
            */
   public void setInjectEnvironmentState(boolean inject_environment_state)
   {
      inject_environment_state_ = inject_environment_state;
   }
   /**
            * Whether to generate environment state information and add it to the prompt
            */
   public boolean getInjectEnvironmentState()
   {
      return inject_environment_state_;
   }

   /**
            * The prompt given once at the very beginning
            */
   public void setSystem(java.lang.String system)
   {
      system_.setLength(0);
      system_.append(system);
   }

   /**
            * The prompt given once at the very beginning
            */
   public java.lang.String getSystemAsString()
   {
      return getSystem().toString();
   }
   /**
            * The prompt given once at the very beginning
            */
   public java.lang.StringBuilder getSystem()
   {
      return system_;
   }

   /**
            * The prompt defining how to make the boolean decision
            */
   public void setPrompt(java.lang.String prompt)
   {
      prompt_.setLength(0);
      prompt_.append(prompt);
   }

   /**
            * The prompt defining how to make the boolean decision
            */
   public java.lang.String getPromptAsString()
   {
      return getPrompt().toString();
   }
   /**
            * The prompt defining how to make the boolean decision
            */
   public java.lang.StringBuilder getPrompt()
   {
      return prompt_;
   }


   public static Supplier<ConditionNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return ConditionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConditionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConditionNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.type_, other.type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.count_to_, other.count_to_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.reset_context_each_run_, other.reset_context_each_run_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.inject_behavior_state_, other.inject_behavior_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.inject_environment_state_, other.inject_environment_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.system_, other.system_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.prompt_, other.prompt_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConditionNodeDefinitionMessage)) return false;

      ConditionNodeDefinitionMessage otherMyClass = (ConditionNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.type_ != otherMyClass.type_) return false;

      if(this.count_to_ != otherMyClass.count_to_) return false;

      if(this.reset_context_each_run_ != otherMyClass.reset_context_each_run_) return false;

      if(this.inject_behavior_state_ != otherMyClass.inject_behavior_state_) return false;

      if(this.inject_environment_state_ != otherMyClass.inject_environment_state_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.system_, otherMyClass.system_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.prompt_, otherMyClass.prompt_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConditionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("count_to=");
      builder.append(this.count_to_);      builder.append(", ");
      builder.append("reset_context_each_run=");
      builder.append(this.reset_context_each_run_);      builder.append(", ");
      builder.append("inject_behavior_state=");
      builder.append(this.inject_behavior_state_);      builder.append(", ");
      builder.append("inject_environment_state=");
      builder.append(this.inject_environment_state_);      builder.append(", ");
      builder.append("system=");
      builder.append(this.system_);      builder.append(", ");
      builder.append("prompt=");
      builder.append(this.prompt_);
      builder.append("}");
      return builder.toString();
   }
}
