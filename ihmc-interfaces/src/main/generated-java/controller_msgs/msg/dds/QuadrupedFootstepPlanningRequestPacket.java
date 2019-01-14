package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped footstep planning module.
       */
public class QuadrupedFootstepPlanningRequestPacket extends Packet<QuadrupedFootstepPlanningRequestPacket> implements Settable<QuadrupedFootstepPlanningRequestPacket>, EpsilonComparable<QuadrupedFootstepPlanningRequestPacket>
{
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.euclid.tuple3D.Point3D body_position_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion body_orientation_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D goal_position_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion goal_orientation_in_world_;
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_message_;
   public int planner_request_id_ = -1;

   public QuadrupedFootstepPlanningRequestPacket()
   {
      body_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      body_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      goal_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      planar_regions_list_message_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();
   }

   public QuadrupedFootstepPlanningRequestPacket(QuadrupedFootstepPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedFootstepPlanningRequestPacket other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.body_position_in_world_, body_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.body_orientation_in_world_, body_orientation_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_position_in_world_, goal_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.goal_orientation_in_world_, goal_orientation_in_world_);
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_message_, planar_regions_list_message_);
      planner_request_id_ = other.planner_request_id_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getBodyPositionInWorld()
   {
      return body_position_in_world_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getBodyOrientationInWorld()
   {
      return body_orientation_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getGoalPositionInWorld()
   {
      return goal_position_in_world_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getGoalOrientationInWorld()
   {
      return goal_orientation_in_world_;
   }


   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsListMessage()
   {
      return planar_regions_list_message_;
   }

   public void setPlannerRequestId(int planner_request_id)
   {
      planner_request_id_ = planner_request_id;
   }
   public int getPlannerRequestId()
   {
      return planner_request_id_;
   }


   public static Supplier<QuadrupedFootstepPlanningRequestPacketPubSubType> getPubSubType()
   {
      return QuadrupedFootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedFootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedFootstepPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.body_position_in_world_.epsilonEquals(other.body_position_in_world_, epsilon)) return false;
      if (!this.body_orientation_in_world_.epsilonEquals(other.body_orientation_in_world_, epsilon)) return false;
      if (!this.goal_position_in_world_.epsilonEquals(other.goal_position_in_world_, epsilon)) return false;
      if (!this.goal_orientation_in_world_.epsilonEquals(other.goal_orientation_in_world_, epsilon)) return false;
      if (!this.planar_regions_list_message_.epsilonEquals(other.planar_regions_list_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedFootstepPlanningRequestPacket)) return false;

      QuadrupedFootstepPlanningRequestPacket otherMyClass = (QuadrupedFootstepPlanningRequestPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.body_position_in_world_.equals(otherMyClass.body_position_in_world_)) return false;
      if (!this.body_orientation_in_world_.equals(otherMyClass.body_orientation_in_world_)) return false;
      if (!this.goal_position_in_world_.equals(otherMyClass.goal_position_in_world_)) return false;
      if (!this.goal_orientation_in_world_.equals(otherMyClass.goal_orientation_in_world_)) return false;
      if (!this.planar_regions_list_message_.equals(otherMyClass.planar_regions_list_message_)) return false;
      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedFootstepPlanningRequestPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("body_position_in_world=");
      builder.append(this.body_position_in_world_);      builder.append(", ");
      builder.append("body_orientation_in_world=");
      builder.append(this.body_orientation_in_world_);      builder.append(", ");
      builder.append("goal_position_in_world=");
      builder.append(this.goal_position_in_world_);      builder.append(", ");
      builder.append("goal_orientation_in_world=");
      builder.append(this.goal_orientation_in_world_);      builder.append(", ");
      builder.append("planar_regions_list_message=");
      builder.append(this.planar_regions_list_message_);      builder.append(", ");
      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);
      builder.append("}");
      return builder.toString();
   }
}
