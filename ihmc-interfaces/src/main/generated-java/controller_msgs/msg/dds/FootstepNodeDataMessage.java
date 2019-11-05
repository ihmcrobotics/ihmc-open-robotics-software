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
public class FootstepNodeDataMessage extends Packet<FootstepNodeDataMessage> implements Settable<FootstepNodeDataMessage>, EpsilonComparable<FootstepNodeDataMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
          * Node rejection reasons. See BipedalFootstepPlannerNodeRejectionReason
          */
   public static final byte STEP_TOO_HIGH_OR_LOW = (byte) 0;
   public static final byte STEP_TOO_FORWARD_AND_DOWN = (byte) 1;
   public static final byte STEP_TOO_FAR = (byte) 2;
   public static final byte STEP_TOO_FAR_AND_HIGH = (byte) 3;
   public static final byte STEP_NOT_WIDE_ENOUGH = (byte) 4;
   public static final byte STEP_IN_PLACE = (byte) 5;
   public static final byte NOT_ENOUGH_AREA = (byte) 6;
   public static final byte COULD_NOT_SNAP = (byte) 7;
   public static final byte COULD_NOT_WIGGLE_INSIDE = (byte) 8;
   public static final byte SURFACE_NORMAL_TOO_STEEP_TO_SNAP = (byte) 9;
   public static final byte TOO_MUCH_PENETRATION_AFTER_WIGGLE = (byte) 10;
   public static final byte STEP_NOT_LONG_ENOUGH = (byte) 11;
   public static final byte STEP_TOO_WIDE = (byte) 12;
   public static final byte OBSTACLE_BLOCKING_BODY = (byte) 13;
   public static final byte OBSTACLE_HITTING_BODY = (byte) 14;
   public static final byte AT_CLIFF_BOTTOM = (byte) 15;
   /**
            * Footstep node side
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Specifies snap transform translation. NaN if snap was unsuccessful
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Specifies the snap rotation. NaN if snap unsuccessful.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   /**
            * ID of parent node. This should reference the index of this node's parent in a FootstepNodeDataList
            */
   public int node_id_ = -1;
   /**
            * ID of node. This should reference the index of this node in a FootstepNodeDataList
            */
   public int parent_node_id_ = -1;
   public int x_index_ = -1;
   public int y_index_ = -1;
   public int yaw_index_ = -1;
   /**
            * Node rejection reason. 255 if node was accepted
            */
   public byte bipedal_footstep_planner_node_rejection_reason_ = (byte) 255;

   public FootstepNodeDataMessage()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public FootstepNodeDataMessage(FootstepNodeDataMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepNodeDataMessage other)
   {
      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      node_id_ = other.node_id_;

      parent_node_id_ = other.parent_node_id_;

      x_index_ = other.x_index_;

      y_index_ = other.y_index_;

      yaw_index_ = other.yaw_index_;

      bipedal_footstep_planner_node_rejection_reason_ = other.bipedal_footstep_planner_node_rejection_reason_;

   }

   /**
            * Footstep node side
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Footstep node side
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Specifies snap transform translation. NaN if snap was unsuccessful
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   /**
            * Specifies the snap rotation. NaN if snap unsuccessful.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }

   /**
            * ID of parent node. This should reference the index of this node's parent in a FootstepNodeDataList
            */
   public void setNodeId(int node_id)
   {
      node_id_ = node_id;
   }
   /**
            * ID of parent node. This should reference the index of this node's parent in a FootstepNodeDataList
            */
   public int getNodeId()
   {
      return node_id_;
   }

   /**
            * ID of node. This should reference the index of this node in a FootstepNodeDataList
            */
   public void setParentNodeId(int parent_node_id)
   {
      parent_node_id_ = parent_node_id;
   }
   /**
            * ID of node. This should reference the index of this node in a FootstepNodeDataList
            */
   public int getParentNodeId()
   {
      return parent_node_id_;
   }

   public void setXIndex(int x_index)
   {
      x_index_ = x_index;
   }
   public int getXIndex()
   {
      return x_index_;
   }

   public void setYIndex(int y_index)
   {
      y_index_ = y_index;
   }
   public int getYIndex()
   {
      return y_index_;
   }

   public void setYawIndex(int yaw_index)
   {
      yaw_index_ = yaw_index;
   }
   public int getYawIndex()
   {
      return yaw_index_;
   }

   /**
            * Node rejection reason. 255 if node was accepted
            */
   public void setBipedalFootstepPlannerNodeRejectionReason(byte bipedal_footstep_planner_node_rejection_reason)
   {
      bipedal_footstep_planner_node_rejection_reason_ = bipedal_footstep_planner_node_rejection_reason;
   }
   /**
            * Node rejection reason. 255 if node was accepted
            */
   public byte getBipedalFootstepPlannerNodeRejectionReason()
   {
      return bipedal_footstep_planner_node_rejection_reason_;
   }


   public static Supplier<FootstepNodeDataMessagePubSubType> getPubSubType()
   {
      return FootstepNodeDataMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepNodeDataMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepNodeDataMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.node_id_, other.node_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.parent_node_id_, other.parent_node_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_index_, other.x_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_index_, other.y_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_index_, other.yaw_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bipedal_footstep_planner_node_rejection_reason_, other.bipedal_footstep_planner_node_rejection_reason_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepNodeDataMessage)) return false;

      FootstepNodeDataMessage otherMyClass = (FootstepNodeDataMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if(this.node_id_ != otherMyClass.node_id_) return false;

      if(this.parent_node_id_ != otherMyClass.parent_node_id_) return false;

      if(this.x_index_ != otherMyClass.x_index_) return false;

      if(this.y_index_ != otherMyClass.y_index_) return false;

      if(this.yaw_index_ != otherMyClass.yaw_index_) return false;

      if(this.bipedal_footstep_planner_node_rejection_reason_ != otherMyClass.bipedal_footstep_planner_node_rejection_reason_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepNodeDataMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("node_id=");
      builder.append(this.node_id_);      builder.append(", ");
      builder.append("parent_node_id=");
      builder.append(this.parent_node_id_);      builder.append(", ");
      builder.append("x_index=");
      builder.append(this.x_index_);      builder.append(", ");
      builder.append("y_index=");
      builder.append(this.y_index_);      builder.append(", ");
      builder.append("yaw_index=");
      builder.append(this.yaw_index_);      builder.append(", ");
      builder.append("bipedal_footstep_planner_node_rejection_reason=");
      builder.append(this.bipedal_footstep_planner_node_rejection_reason_);
      builder.append("}");
      return builder.toString();
   }
}
