package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DoorTraversalDefinitionMessage extends Packet<DoorTraversalDefinitionMessage> implements Settable<DoorTraversalDefinitionMessage>, EpsilonComparable<DoorTraversalDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;

   public DoorTraversalDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public DoorTraversalDefinitionMessage(DoorTraversalDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorTraversalDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<DoorTraversalDefinitionMessagePubSubType> getPubSubType()
   {
      return DoorTraversalDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorTraversalDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorTraversalDefinitionMessage other, double epsilon)
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
      if(!(other instanceof DoorTraversalDefinitionMessage)) return false;

      DoorTraversalDefinitionMessage otherMyClass = (DoorTraversalDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorTraversalDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
