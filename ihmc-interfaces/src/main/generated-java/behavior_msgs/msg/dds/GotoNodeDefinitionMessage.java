package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GotoNodeDefinitionMessage extends Packet<GotoNodeDefinitionMessage> implements Settable<GotoNodeDefinitionMessage>, EpsilonComparable<GotoNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;

   public GotoNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public GotoNodeDefinitionMessage(GotoNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(GotoNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<GotoNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return GotoNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GotoNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GotoNodeDefinitionMessage other, double epsilon)
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
      if(!(other instanceof GotoNodeDefinitionMessage)) return false;

      GotoNodeDefinitionMessage otherMyClass = (GotoNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GotoNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
