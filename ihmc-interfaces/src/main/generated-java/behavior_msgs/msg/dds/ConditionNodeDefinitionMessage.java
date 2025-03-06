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
            * The prompt defining how to make the boolean decision
            */
   public java.lang.StringBuilder prompt_;

   public ConditionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.LeafNodeDefinitionMessage();
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
      builder.append("prompt=");
      builder.append(this.prompt_);
      builder.append("}");
      return builder.toString();
   }
}
