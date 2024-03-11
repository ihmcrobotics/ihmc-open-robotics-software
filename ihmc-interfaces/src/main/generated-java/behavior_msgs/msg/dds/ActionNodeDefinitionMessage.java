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
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * If the node is set to execute after the previous node
            */
   public boolean execute_after_previous_;
   /**
            * If the node is set to execute after the beginning
            */
   public boolean execute_after_beginning_;
   /**
            * The ID of the node to execute after
            */
   public int execute_after_node_id_;

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
      execute_after_previous_ = other.execute_after_previous_;

      execute_after_beginning_ = other.execute_after_beginning_;

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
            * If the node is set to execute after the previous node
            */
   public void setExecuteAfterPrevious(boolean execute_after_previous)
   {
      execute_after_previous_ = execute_after_previous;
   }
   /**
            * If the node is set to execute after the previous node
            */
   public boolean getExecuteAfterPrevious()
   {
      return execute_after_previous_;
   }

   /**
            * If the node is set to execute after the beginning
            */
   public void setExecuteAfterBeginning(boolean execute_after_beginning)
   {
      execute_after_beginning_ = execute_after_beginning;
   }
   /**
            * If the node is set to execute after the beginning
            */
   public boolean getExecuteAfterBeginning()
   {
      return execute_after_beginning_;
   }

   /**
            * The ID of the node to execute after
            */
   public void setExecuteAfterNodeId(int execute_after_node_id)
   {
      execute_after_node_id_ = execute_after_node_id;
   }
   /**
            * The ID of the node to execute after
            */
   public int getExecuteAfterNodeId()
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_after_previous_, other.execute_after_previous_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_after_beginning_, other.execute_after_beginning_, epsilon)) return false;

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
      if(this.execute_after_previous_ != otherMyClass.execute_after_previous_) return false;

      if(this.execute_after_beginning_ != otherMyClass.execute_after_beginning_) return false;

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
      builder.append("execute_after_previous=");
      builder.append(this.execute_after_previous_);      builder.append(", ");
      builder.append("execute_after_beginning=");
      builder.append(this.execute_after_beginning_);      builder.append(", ");
      builder.append("execute_after_node_id=");
      builder.append(this.execute_after_node_id_);
      builder.append("}");
      return builder.toString();
   }
}
