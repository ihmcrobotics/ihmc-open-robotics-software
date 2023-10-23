package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorActionSequenceDefinitionMessage extends Packet<BehaviorActionSequenceDefinitionMessage> implements Settable<BehaviorActionSequenceDefinitionMessage>, EpsilonComparable<BehaviorActionSequenceDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;

   public BehaviorActionSequenceDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public BehaviorActionSequenceDefinitionMessage(BehaviorActionSequenceDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorActionSequenceDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<BehaviorActionSequenceDefinitionMessagePubSubType> getPubSubType()
   {
      return BehaviorActionSequenceDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorActionSequenceDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorActionSequenceDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorActionSequenceDefinitionMessage)) return false;

      BehaviorActionSequenceDefinitionMessage otherMyClass = (BehaviorActionSequenceDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorActionSequenceDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
