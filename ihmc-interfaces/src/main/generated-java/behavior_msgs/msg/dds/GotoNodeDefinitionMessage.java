package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GotoNodeDefinitionMessage extends Packet<GotoNodeDefinitionMessage> implements Settable<GotoNodeDefinitionMessage>, EpsilonComparable<GotoNodeDefinitionMessage>
{
   /**
          * Value of the node to goto ID when we have a name but still need to find the ID
          */
   public static final long INVALID = -1;
   /**
          * Value of the node to goto ID when the node is simply set to goto the next node (i.e. no-op)
          */
   public static final long GOTO_NEXT = -2;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage definition_;
   /**
            * The ID of the node to goto or one of the invalid or next constants
            */
   public long node_to_goto_id_;

   public GotoNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.LeafNodeDefinitionMessage();
   }

   public GotoNodeDefinitionMessage(GotoNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(GotoNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      node_to_goto_id_ = other.node_to_goto_id_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The ID of the node to goto or one of the invalid or next constants
            */
   public void setNodeToGotoId(long node_to_goto_id)
   {
      node_to_goto_id_ = node_to_goto_id;
   }
   /**
            * The ID of the node to goto or one of the invalid or next constants
            */
   public long getNodeToGotoId()
   {
      return node_to_goto_id_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.node_to_goto_id_, other.node_to_goto_id_, epsilon)) return false;


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
      if(this.node_to_goto_id_ != otherMyClass.node_to_goto_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GotoNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("node_to_goto_id=");
      builder.append(this.node_to_goto_id_);
      builder.append("}");
      return builder.toString();
   }
}
