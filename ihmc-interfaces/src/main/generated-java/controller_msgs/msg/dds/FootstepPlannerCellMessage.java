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
public class FootstepPlannerCellMessage extends Packet<FootstepPlannerCellMessage> implements Settable<FootstepPlannerCellMessage>, EpsilonComparable<FootstepPlannerCellMessage>
{
   /**
            * X index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public int x_index_;
   /**
            * Y index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public int y_index_;
   public boolean node_is_valid_ = true;

   public FootstepPlannerCellMessage()
   {
   }

   public FootstepPlannerCellMessage(FootstepPlannerCellMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerCellMessage other)
   {
      x_index_ = other.x_index_;

      y_index_ = other.y_index_;

      node_is_valid_ = other.node_is_valid_;

   }

   /**
            * X index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public void setXIndex(int x_index)
   {
      x_index_ = x_index;
   }
   /**
            * X index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public int getXIndex()
   {
      return x_index_;
   }

   /**
            * Y index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public void setYIndex(int y_index)
   {
      y_index_ = y_index;
   }
   /**
            * Y index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public int getYIndex()
   {
      return y_index_;
   }

   public void setNodeIsValid(boolean node_is_valid)
   {
      node_is_valid_ = node_is_valid;
   }
   public boolean getNodeIsValid()
   {
      return node_is_valid_;
   }


   public static Supplier<FootstepPlannerCellMessagePubSubType> getPubSubType()
   {
      return FootstepPlannerCellMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerCellMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerCellMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_index_, other.x_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_index_, other.y_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.node_is_valid_, other.node_is_valid_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerCellMessage)) return false;

      FootstepPlannerCellMessage otherMyClass = (FootstepPlannerCellMessage) other;

      if(this.x_index_ != otherMyClass.x_index_) return false;

      if(this.y_index_ != otherMyClass.y_index_) return false;

      if(this.node_is_valid_ != otherMyClass.node_is_valid_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerCellMessage {");
      builder.append("x_index=");
      builder.append(this.x_index_);      builder.append(", ");
      builder.append("y_index=");
      builder.append(this.y_index_);      builder.append(", ");
      builder.append("node_is_valid=");
      builder.append(this.node_is_valid_);
      builder.append("}");
      return builder.toString();
   }
}
