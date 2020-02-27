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
   /**
          * Node rejection reasons. See BipedalFootstepPlannerNodeRejectionReason
          */
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_HIGH_OR_LOW = (byte) 0;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED = (byte) 1;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_FORWARD_AND_DOWN = (byte) 2;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_WIDE_AND_DOWN = (byte) 3;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_FAR = (byte) 4;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_FAR_AND_HIGH = (byte) 5;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_WIDE_AND_HIGH = (byte) 6;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_NOT_WIDE_ENOUGH = (byte) 7;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_IN_PLACE = (byte) 8;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_NOT_ENOUGH_AREA = (byte) 9;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_COULD_NOT_SNAP = (byte) 10;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_SURFACE_NORMAL_TOO_STEEP_TO_SNAP = (byte) 11;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_TOO_MUCH_PENETRATION_AFTER_WIGGLE = (byte) 12;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_NOT_LONG_ENOUGH = (byte) 13;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_WIDE = (byte) 14;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_OBSTACLE_BLOCKING_BODY = (byte) 15;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_OBSTACLE_HITTING_BODY = (byte) 16;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_AT_CLIFF_BOTTOM = (byte) 17;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_YAWS_TOO_MUCH = (byte) 18;
   /**
            * Specifies snap transform translation. NaN if snap was unsuccessful
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Specifies the snap rotation. NaN if snap unsuccessful.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   /**
            * ID of parent node. This should reference the index of this node in a FootstepNodeDataList
            */
   public int parent_node_id_ = -1;
   public controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage footstep_node_;
   /**
            * Node rejection reason. 255 if node was accepted
            */
   public byte bipedal_footstep_planner_node_rejection_reason_ = (byte) 255;

   public FootstepNodeDataMessage()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      footstep_node_ = new controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage();
   }

   public FootstepNodeDataMessage(FootstepNodeDataMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepNodeDataMessage other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      parent_node_id_ = other.parent_node_id_;

      controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessagePubSubType.staticCopy(other.footstep_node_, footstep_node_);
      bipedal_footstep_planner_node_rejection_reason_ = other.bipedal_footstep_planner_node_rejection_reason_;

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
            * ID of parent node. This should reference the index of this node in a FootstepNodeDataList
            */
   public void setParentNodeId(int parent_node_id)
   {
      parent_node_id_ = parent_node_id;
   }
   /**
            * ID of parent node. This should reference the index of this node in a FootstepNodeDataList
            */
   public int getParentNodeId()
   {
      return parent_node_id_;
   }


   public controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage getFootstepNode()
   {
      return footstep_node_;
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

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.parent_node_id_, other.parent_node_id_, epsilon)) return false;

      if (!this.footstep_node_.epsilonEquals(other.footstep_node_, epsilon)) return false;
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

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if(this.parent_node_id_ != otherMyClass.parent_node_id_) return false;

      if (!this.footstep_node_.equals(otherMyClass.footstep_node_)) return false;
      if(this.bipedal_footstep_planner_node_rejection_reason_ != otherMyClass.bipedal_footstep_planner_node_rejection_reason_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepNodeDataMessage {");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("parent_node_id=");
      builder.append(this.parent_node_id_);      builder.append(", ");
      builder.append("footstep_node=");
      builder.append(this.footstep_node_);      builder.append(", ");
      builder.append("bipedal_footstep_planner_node_rejection_reason=");
      builder.append(this.bipedal_footstep_planner_node_rejection_reason_);
      builder.append("}");
      return builder.toString();
   }
}
