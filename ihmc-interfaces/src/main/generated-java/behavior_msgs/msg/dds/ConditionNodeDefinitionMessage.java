package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ConditionNodeDefinitionMessage extends Packet<ConditionNodeDefinitionMessage> implements Settable<ConditionNodeDefinitionMessage>, EpsilonComparable<ConditionNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * The number of times to fail before passing
            */
   public long count_to_;

   public ConditionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public ConditionNodeDefinitionMessage(ConditionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ConditionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      count_to_ = other.count_to_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.count_to_, other.count_to_, epsilon)) return false;


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
      if(this.count_to_ != otherMyClass.count_to_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConditionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("count_to=");
      builder.append(this.count_to_);
      builder.append("}");
      return builder.toString();
   }
}
