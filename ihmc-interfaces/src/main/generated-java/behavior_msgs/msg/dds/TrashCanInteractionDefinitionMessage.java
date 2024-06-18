package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TrashCanInteractionDefinitionMessage extends Packet<TrashCanInteractionDefinitionMessage> implements Settable<TrashCanInteractionDefinitionMessage>, EpsilonComparable<TrashCanInteractionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   public java.lang.StringBuilder obstructed_node_name_;

   public TrashCanInteractionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
      obstructed_node_name_ = new java.lang.StringBuilder(255);
   }

   public TrashCanInteractionDefinitionMessage(TrashCanInteractionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(TrashCanInteractionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      obstructed_node_name_.setLength(0);
      obstructed_node_name_.append(other.obstructed_node_name_);

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   public void setObstructedNodeName(java.lang.String obstructed_node_name)
   {
      obstructed_node_name_.setLength(0);
      obstructed_node_name_.append(obstructed_node_name);
   }

   public java.lang.String getObstructedNodeNameAsString()
   {
      return getObstructedNodeName().toString();
   }
   public java.lang.StringBuilder getObstructedNodeName()
   {
      return obstructed_node_name_;
   }


   public static Supplier<TrashCanInteractionDefinitionMessagePubSubType> getPubSubType()
   {
      return TrashCanInteractionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TrashCanInteractionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TrashCanInteractionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.obstructed_node_name_, other.obstructed_node_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TrashCanInteractionDefinitionMessage)) return false;

      TrashCanInteractionDefinitionMessage otherMyClass = (TrashCanInteractionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.obstructed_node_name_, otherMyClass.obstructed_node_name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TrashCanInteractionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("obstructed_node_name=");
      builder.append(this.obstructed_node_name_);
      builder.append("}");
      return builder.toString();
   }
}
