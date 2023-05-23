package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeControlFlowNodeMessage extends Packet<BehaviorTreeControlFlowNodeMessage> implements Settable<BehaviorTreeControlFlowNodeMessage>, EpsilonComparable<BehaviorTreeControlFlowNodeMessage>
{
   /**
            * The number of children
            * This is used for a stack data structure to re-assemble the tree.
            */
   public long number_of_children_;
   /**
            * Last tick instant
            */
   public ihmc_common_msgs.msg.dds.InstantMessage last_tick_instant_;
   /**
            * Name of the node
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  node_name_;
   /**
            * The type of the node, as a string
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  node_type_;
   /**
            * Previous node status
            */
   public byte previous_status_;
   /**
            * Whether this node has been clocked
            */
   public boolean has_been_clocked_;

   public BehaviorTreeControlFlowNodeMessage()
   {
      last_tick_instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      node_name_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (256, "type_d");
      node_type_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (256, "type_d");
   }

   public BehaviorTreeControlFlowNodeMessage(BehaviorTreeControlFlowNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeControlFlowNodeMessage other)
   {
      number_of_children_ = other.number_of_children_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_tick_instant_, last_tick_instant_);
      node_name_.set(other.node_name_);
      node_type_.set(other.node_type_);
      previous_status_ = other.previous_status_;

      has_been_clocked_ = other.has_been_clocked_;

   }

   /**
            * The number of children
            * This is used for a stack data structure to re-assemble the tree.
            */
   public void setNumberOfChildren(long number_of_children)
   {
      number_of_children_ = number_of_children;
   }
   /**
            * The number of children
            * This is used for a stack data structure to re-assemble the tree.
            */
   public long getNumberOfChildren()
   {
      return number_of_children_;
   }


   /**
            * Last tick instant
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getLastTickInstant()
   {
      return last_tick_instant_;
   }


   /**
            * Name of the node
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getNodeName()
   {
      return node_name_;
   }


   /**
            * The type of the node, as a string
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getNodeType()
   {
      return node_type_;
   }

   /**
            * Previous node status
            */
   public void setPreviousStatus(byte previous_status)
   {
      previous_status_ = previous_status;
   }
   /**
            * Previous node status
            */
   public byte getPreviousStatus()
   {
      return previous_status_;
   }

   /**
            * Whether this node has been clocked
            */
   public void setHasBeenClocked(boolean has_been_clocked)
   {
      has_been_clocked_ = has_been_clocked;
   }
   /**
            * Whether this node has been clocked
            */
   public boolean getHasBeenClocked()
   {
      return has_been_clocked_;
   }


   public static Supplier<BehaviorTreeControlFlowNodeMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeControlFlowNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeControlFlowNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeControlFlowNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_children_, other.number_of_children_, epsilon)) return false;

      if (!this.last_tick_instant_.epsilonEquals(other.last_tick_instant_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.node_name_, other.node_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.node_type_, other.node_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.previous_status_, other.previous_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_been_clocked_, other.has_been_clocked_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeControlFlowNodeMessage)) return false;

      BehaviorTreeControlFlowNodeMessage otherMyClass = (BehaviorTreeControlFlowNodeMessage) other;

      if(this.number_of_children_ != otherMyClass.number_of_children_) return false;

      if (!this.last_tick_instant_.equals(otherMyClass.last_tick_instant_)) return false;
      if (!this.node_name_.equals(otherMyClass.node_name_)) return false;
      if (!this.node_type_.equals(otherMyClass.node_type_)) return false;
      if(this.previous_status_ != otherMyClass.previous_status_) return false;

      if(this.has_been_clocked_ != otherMyClass.has_been_clocked_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeControlFlowNodeMessage {");
      builder.append("number_of_children=");
      builder.append(this.number_of_children_);      builder.append(", ");
      builder.append("last_tick_instant=");
      builder.append(this.last_tick_instant_);      builder.append(", ");
      builder.append("node_name=");
      builder.append(this.node_name_);      builder.append(", ");
      builder.append("node_type=");
      builder.append(this.node_type_);      builder.append(", ");
      builder.append("previous_status=");
      builder.append(this.previous_status_);      builder.append(", ");
      builder.append("has_been_clocked=");
      builder.append(this.has_been_clocked_);
      builder.append("}");
      return builder.toString();
   }
}
