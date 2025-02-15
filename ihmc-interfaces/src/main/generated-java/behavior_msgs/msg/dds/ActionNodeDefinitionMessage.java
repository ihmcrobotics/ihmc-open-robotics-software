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
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage definition_;

   public ActionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.LeafNodeDefinitionMessage();
   }

   public ActionNodeDefinitionMessage(ActionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage getDefinition()
   {
      return definition_;
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

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
