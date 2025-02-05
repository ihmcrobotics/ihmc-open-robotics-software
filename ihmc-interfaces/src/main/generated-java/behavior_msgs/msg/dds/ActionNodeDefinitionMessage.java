package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionNodeDefinitionMessage extends Packet<ActionNodeDefinitionMessage> implements Settable<ActionNodeDefinitionMessage>, EpsilonComparable<ActionNodeDefinitionMessage>
{
   /**
          * Value of execute after node ID when we have a name but still need to find the ID
          */
   public static final long INVALID = -1;
   /**
          * Value of execute after node ID when this action is set to execute after the previous node
          */
   public static final long EXECUTE_AFTER_PREVIOUS = -2;
   /**
          * Value of execute after node ID when this action is set to execute after the beginning of the sequence
          */
   public static final long EXECUTE_AFTER_BEGINNING = -3;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * The ID of the node to execute after or one of the invalid, previous, or beginning constants
            */
   public long execute_after_node_id_;

   public ActionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public ActionNodeDefinitionMessage(ActionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      execute_after_node_id_ = other.execute_after_node_id_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The ID of the node to execute after or one of the invalid, previous, or beginning constants
            */
   public void setExecuteAfterNodeId(long execute_after_node_id)
   {
      execute_after_node_id_ = execute_after_node_id;
   }
   /**
            * The ID of the node to execute after or one of the invalid, previous, or beginning constants
            */
   public long getExecuteAfterNodeId()
   {
      return execute_after_node_id_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execute_after_node_id_, other.execute_after_node_id_, epsilon)) return false;


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
      if(this.execute_after_node_id_ != otherMyClass.execute_after_node_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("execute_after_node_id=");
      builder.append(this.execute_after_node_id_);
      builder.append("}");
      return builder.toString();
   }
}
