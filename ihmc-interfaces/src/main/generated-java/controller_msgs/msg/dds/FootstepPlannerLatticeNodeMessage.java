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
public class FootstepPlannerLatticeNodeMessage extends Packet<FootstepPlannerLatticeNodeMessage> implements Settable<FootstepPlannerLatticeNodeMessage>, EpsilonComparable<FootstepPlannerLatticeNodeMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * X index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public int x_index_;
   /**
            * Y index of the cell. See us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode for more information
            */
   public int y_index_;
   /**
            * Yaw index of the cell
            */
   public int yaw_index_;
   /**
            * Robot side of the cell
            */
   public byte robot_side_;
   public int node_index_;

   public FootstepPlannerLatticeNodeMessage()
   {
   }

   public FootstepPlannerLatticeNodeMessage(FootstepPlannerLatticeNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerLatticeNodeMessage other)
   {
      x_index_ = other.x_index_;

      y_index_ = other.y_index_;

      yaw_index_ = other.yaw_index_;

      robot_side_ = other.robot_side_;

      node_index_ = other.node_index_;

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

   /**
            * Yaw index of the cell
            */
   public void setYawIndex(int yaw_index)
   {
      yaw_index_ = yaw_index;
   }
   /**
            * Yaw index of the cell
            */
   public int getYawIndex()
   {
      return yaw_index_;
   }

   /**
            * Robot side of the cell
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Robot side of the cell
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setNodeIndex(int node_index)
   {
      node_index_ = node_index;
   }
   public int getNodeIndex()
   {
      return node_index_;
   }


   public static Supplier<FootstepPlannerLatticeNodeMessagePubSubType> getPubSubType()
   {
      return FootstepPlannerLatticeNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerLatticeNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerLatticeNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_index_, other.x_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_index_, other.y_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_index_, other.yaw_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.node_index_, other.node_index_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerLatticeNodeMessage)) return false;

      FootstepPlannerLatticeNodeMessage otherMyClass = (FootstepPlannerLatticeNodeMessage) other;

      if(this.x_index_ != otherMyClass.x_index_) return false;

      if(this.y_index_ != otherMyClass.y_index_) return false;

      if(this.yaw_index_ != otherMyClass.yaw_index_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.node_index_ != otherMyClass.node_index_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerLatticeNodeMessage {");
      builder.append("x_index=");
      builder.append(this.x_index_);      builder.append(", ");
      builder.append("y_index=");
      builder.append(this.y_index_);      builder.append(", ");
      builder.append("yaw_index=");
      builder.append(this.yaw_index_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("node_index=");
      builder.append(this.node_index_);
      builder.append("}");
      return builder.toString();
   }
}
