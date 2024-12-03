package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FallbackNodeDefinitionMessage extends Packet<FallbackNodeDefinitionMessage> implements Settable<FallbackNodeDefinitionMessage>, EpsilonComparable<FallbackNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * The ID of the action to retry
            */
   public long goto_action_id_;

   public FallbackNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public FallbackNodeDefinitionMessage(FallbackNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(FallbackNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      goto_action_id_ = other.goto_action_id_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The ID of the action to retry
            */
   public void setGotoActionId(long goto_action_id)
   {
      goto_action_id_ = goto_action_id;
   }
   /**
            * The ID of the action to retry
            */
   public long getGotoActionId()
   {
      return goto_action_id_;
   }


   public static Supplier<FallbackNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return FallbackNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FallbackNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FallbackNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goto_action_id_, other.goto_action_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FallbackNodeDefinitionMessage)) return false;

      FallbackNodeDefinitionMessage otherMyClass = (FallbackNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.goto_action_id_ != otherMyClass.goto_action_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FallbackNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("goto_action_id=");
      builder.append(this.goto_action_id_);
      builder.append("}");
      return builder.toString();
   }
}
