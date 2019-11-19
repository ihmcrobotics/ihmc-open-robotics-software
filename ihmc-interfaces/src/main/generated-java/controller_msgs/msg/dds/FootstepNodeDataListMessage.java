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
   public long plan_id_;
   /**
            * List of all footstep node data. Index 0 should hold the start node
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepNodeDataMessage>  node_data_;
   /**
            * This message has two modes:
            * Mode 1: The planner will periodically emit a list of nodes representing the current lowest cost path while planning.
            * The interpretation of lower cost path depends on the planner, but usually means the sequence of nodes with the lowest ((cost from start) + heuristic) value.
            * In this case, each of the node_data elements will not have the rejection reason or parent id fields populated
            * Mode 2: The planner can emit the entire footstep graph, with rejection reasons and parents indices populated.
            * This is emitted once after planning has completed.
            * If this value is true, this packet is in mode 2
            */
   public boolean is_footstep_graph_;

   public FootstepNodeDataListMessage()
   {
      node_data_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepNodeDataMessage> (200000, new controller_msgs.msg.dds.FootstepNodeDataMessagePubSubType());

   }

   public FootstepNodeDataListMessage(FootstepNodeDataListMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepNodeDataListMessage other)
   {
      plan_id_ = other.plan_id_;

      node_data_.set(other.node_data_);
      is_footstep_graph_ = other.is_footstep_graph_;

   }

   /**
            * This is the ID of the planning request which this message corresponds to
            */
   public void setPlanId(long plan_id)
   {
      plan_id_ = plan_id;
   }
   /**
            * This is the ID of the planning request which this message corresponds to
            */
   public long getPlanId()
   {
      return plan_id_;
   }


   /**
            * List of all footstep node data. Index 0 should hold the start node
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepNodeDataMessage>  getNodeData()
   {
      return node_data_;
   }

   /**
            * This message has two modes:
            * Mode 1: The planner will periodically emit a list of nodes representing the current lowest cost path while planning.
            * The interpretation of lower cost path depends on the planner, but usually means the sequence of nodes with the lowest ((cost from start) + heuristic) value.
            * In this case, each of the node_data elements will not have the rejection reason or parent id fields populated
            * Mode 2: The planner can emit the entire footstep graph, with rejection reasons and parents indices populated.
            * This is emitted once after planning has completed.
            * If this value is true, this packet is in mode 2
            */
   public void setIsFootstepGraph(boolean is_footstep_graph)
   {
      is_footstep_graph_ = is_footstep_graph;
   }
   /**
            * This message has two modes:
            * Mode 1: The planner will periodically emit a list of nodes representing the current lowest cost path while planning.
            * The interpretation of lower cost path depends on the planner, but usually means the sequence of nodes with the lowest ((cost from start) + heuristic) value.
            * In this case, each of the node_data elements will not have the rejection reason or parent id fields populated
            * Mode 2: The planner can emit the entire footstep graph, with rejection reasons and parents indices populated.
            * This is emitted once after planning has completed.
            * If this value is true, this packet is in mode 2
            */
   public boolean getIsFootstepGraph()
   {
      return is_footstep_graph_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (this.node_data_.size() != other.node_data_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.node_data_.size(); i++)
         {  if (!this.node_data_.get(i).epsilonEquals(other.node_data_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_footstep_graph_, other.is_footstep_graph_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepNodeDataListMessage)) return false;

      FootstepNodeDataListMessage otherMyClass = (FootstepNodeDataListMessage) other;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.node_data_.equals(otherMyClass.node_data_)) return false;
      if(this.is_footstep_graph_ != otherMyClass.is_footstep_graph_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepNodeDataListMessage {");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("node_data=");
      builder.append(this.node_data_);      builder.append(", ");
      builder.append("is_footstep_graph=");
      builder.append(this.is_footstep_graph_);
      builder.append("}");
      return builder.toString();
   }
}
