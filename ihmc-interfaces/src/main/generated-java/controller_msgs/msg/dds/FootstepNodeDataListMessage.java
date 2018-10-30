package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       * This message is sent from the planner in order to visualize and debug planner progress
       */
public class FootstepNodeDataListMessage extends Packet<FootstepNodeDataListMessage> implements Settable<FootstepNodeDataListMessage>, EpsilonComparable<FootstepNodeDataListMessage>
{
   /**
            * This is the ID of the planning request which this message corresponds to
            */
   public long sequence_id_;
   /**
            * List of all footstep node data. Index 0 should hold the start node
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepNodeDataMessage>  node_data_;

   public FootstepNodeDataListMessage()
   {
      node_data_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepNodeDataMessage> (100, new controller_msgs.msg.dds.FootstepNodeDataMessagePubSubType());

   }

   public FootstepNodeDataListMessage(FootstepNodeDataListMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepNodeDataListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      node_data_.set(other.node_data_);
   }

   /**
            * This is the ID of the planning request which this message corresponds to
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * This is the ID of the planning request which this message corresponds to
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   /**
            * List of all footstep node data. Index 0 should hold the start node
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepNodeDataMessage>  getNodeData()
   {
      return node_data_;
   }


   public static Supplier<FootstepNodeDataListMessagePubSubType> getPubSubType()
   {
      return FootstepNodeDataListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepNodeDataListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepNodeDataListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.node_data_.size() != other.node_data_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.node_data_.size(); i++)
         {  if (!this.node_data_.get(i).epsilonEquals(other.node_data_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepNodeDataListMessage)) return false;

      FootstepNodeDataListMessage otherMyClass = (FootstepNodeDataListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.node_data_.equals(otherMyClass.node_data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepNodeDataListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("node_data=");
      builder.append(this.node_data_);
      builder.append("}");
      return builder.toString();
   }
}
