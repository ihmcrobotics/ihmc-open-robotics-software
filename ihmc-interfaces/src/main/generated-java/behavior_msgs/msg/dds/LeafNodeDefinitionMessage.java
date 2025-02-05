package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LeafNodeDefinitionMessage extends Packet<LeafNodeDefinitionMessage> implements Settable<LeafNodeDefinitionMessage>, EpsilonComparable<LeafNodeDefinitionMessage>
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
            * The ID of the node to execute after or one of the invalid, previous, or beginning constants
            */
   public long execute_after_node_id_;

   public LeafNodeDefinitionMessage()
   {
   }

   public LeafNodeDefinitionMessage(LeafNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(LeafNodeDefinitionMessage other)
   {
      execute_after_node_id_ = other.execute_after_node_id_;

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


   public static Supplier<LeafNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return LeafNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LeafNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LeafNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execute_after_node_id_, other.execute_after_node_id_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LeafNodeDefinitionMessage)) return false;

      LeafNodeDefinitionMessage otherMyClass = (LeafNodeDefinitionMessage) other;

      if(this.execute_after_node_id_ != otherMyClass.execute_after_node_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LeafNodeDefinitionMessage {");
      builder.append("execute_after_node_id=");
      builder.append(this.execute_after_node_id_);
      builder.append("}");
      return builder.toString();
   }
}
