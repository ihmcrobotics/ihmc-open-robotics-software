package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlanningRequestPacket extends Packet<FootstepPlanningRequestPacket> implements Settable<FootstepPlanningRequestPacket>, EpsilonComparable<FootstepPlanningRequestPacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public static final byte FOOTSTEP_PLANNER_TYPE_PLANAR_REGION_BIPEDAL = (byte) 0;
   public static final byte FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP = (byte) 1;
   /**
          * The recommended planner type
          */
   public static final byte FOOTSTEP_PLANNER_TYPE_A_STAR = (byte) 2;
   public static final byte FOOTSTEP_PLANNER_TYPE_SIMPLE_BODY_PATH = (byte) 3;
   public static final byte FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR = (byte) 4;
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Initial stance foot side
            */
   public byte initial_stance_robot_side_ = (byte) 255;
   /**
            * Stance foot sole frame z up position, z must be correct
            */
   public us.ihmc.euclid.tuple3D.Point3D stance_foot_position_in_world_;
   /**
            * Stance foot sole frame z up orientation, only the "yaw" is considered
            */
   public us.ihmc.euclid.tuple4D.Quaternion stance_foot_orientation_in_world_;
   /**
            * Goal mid feet z up position
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_position_in_world_;
   /**
            * Goal mid feet z up orientation, only the "yaw" is considered
            */
   public us.ihmc.euclid.tuple4D.Quaternion goal_orientation_in_world_;
   /**
            * Footstep planner type, see above
            */
   public byte requested_footstep_planner_type_ = (byte) 255;
   /**
            * Timeout in seconds
            */
   public double timeout_;
   /**
            * Max body path length if using body path
            */
   public double horizon_length_;
   /**
            * Planar regions to use, if you don't want to assume flat ground
            */
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_message_;
   /**
            * Explicitly tell the planner to use flat ground
            */
   public boolean assume_flat_ground_;
   /**
            * Set this id to keep track of your request
            */
   public int planner_request_id_ = -1;

   public FootstepPlanningRequestPacket()
   {
      stance_foot_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      stance_foot_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      goal_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      planar_regions_list_message_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();
   }

   public FootstepPlanningRequestPacket(FootstepPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanningRequestPacket other)
   {
      sequence_id_ = other.sequence_id_;

      initial_stance_robot_side_ = other.initial_stance_robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.stance_foot_position_in_world_, stance_foot_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.stance_foot_orientation_in_world_, stance_foot_orientation_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_position_in_world_, goal_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.goal_orientation_in_world_, goal_orientation_in_world_);
      requested_footstep_planner_type_ = other.requested_footstep_planner_type_;

      timeout_ = other.timeout_;

      horizon_length_ = other.horizon_length_;

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_message_, planar_regions_list_message_);
      assume_flat_ground_ = other.assume_flat_ground_;

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

   /**
            * Initial stance foot side
            */
   public void setInitialStanceRobotSide(byte initial_stance_robot_side)
   {
      initial_stance_robot_side_ = initial_stance_robot_side;
   }
   /**
            * Initial stance foot side
            */
   public byte getInitialStanceRobotSide()
   {
      return initial_stance_robot_side_;
   }


   /**
            * Stance foot sole frame z up position, z must be correct
            */
   public us.ihmc.euclid.tuple3D.Point3D getStanceFootPositionInWorld()
   {
      return stance_foot_position_in_world_;
   }


   /**
            * Stance foot sole frame z up orientation, only the "yaw" is considered
            */
   public us.ihmc.euclid.tuple4D.Quaternion getStanceFootOrientationInWorld()
   {
      return stance_foot_orientation_in_world_;
   }


   /**
            * Goal mid feet z up position
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalPositionInWorld()
   {
      return goal_position_in_world_;
   }


   /**
            * Goal mid feet z up orientation, only the "yaw" is considered
            */
   public us.ihmc.euclid.tuple4D.Quaternion getGoalOrientationInWorld()
   {
      return goal_orientation_in_world_;
   }

   /**
            * Footstep planner type, see above
            */
   public void setRequestedFootstepPlannerType(byte requested_footstep_planner_type)
   {
      requested_footstep_planner_type_ = requested_footstep_planner_type;
   }
   /**
            * Footstep planner type, see above
            */
   public byte getRequestedFootstepPlannerType()
   {
      return requested_footstep_planner_type_;
   }

   /**
            * Timeout in seconds
            */
   public void setTimeout(double timeout)
   {
      timeout_ = timeout;
   }
   /**
            * Timeout in seconds
            */
   public double getTimeout()
   {
      return timeout_;
   }

   /**
            * Max body path length if using body path
            */
   public void setHorizonLength(double horizon_length)
   {
      horizon_length_ = horizon_length;
   }
   /**
            * Max body path length if using body path
            */
   public double getHorizonLength()
   {
      return horizon_length_;
   }


   /**
            * Planar regions to use, if you don't want to assume flat ground
            */
   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsListMessage()
   {
      return planar_regions_list_message_;
   }

   /**
            * Explicitly tell the planner to use flat ground
            */
   public void setAssumeFlatGround(boolean assume_flat_ground)
   {
      assume_flat_ground_ = assume_flat_ground;
   }
   /**
            * Explicitly tell the planner to use flat ground
            */
   public boolean getAssumeFlatGround()
   {
      return assume_flat_ground_;
   }

   /**
            * Set this id to keep track of your request
            */
   public void setPlannerRequestId(int planner_request_id)
   {
      planner_request_id_ = planner_request_id;
   }
   /**
            * Set this id to keep track of your request
            */
   public int getPlannerRequestId()
   {
      return planner_request_id_;
   }


   public static Supplier<FootstepPlanningRequestPacketPubSubType> getPubSubType()
   {
      return FootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_stance_robot_side_, other.initial_stance_robot_side_, epsilon)) return false;

      if (!this.stance_foot_position_in_world_.epsilonEquals(other.stance_foot_position_in_world_, epsilon)) return false;
      if (!this.stance_foot_orientation_in_world_.epsilonEquals(other.stance_foot_orientation_in_world_, epsilon)) return false;
      if (!this.goal_position_in_world_.epsilonEquals(other.goal_position_in_world_, epsilon)) return false;
      if (!this.goal_orientation_in_world_.epsilonEquals(other.goal_orientation_in_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_footstep_planner_type_, other.requested_footstep_planner_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.horizon_length_, other.horizon_length_, epsilon)) return false;

      if (!this.planar_regions_list_message_.epsilonEquals(other.planar_regions_list_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.assume_flat_ground_, other.assume_flat_ground_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanningRequestPacket)) return false;

      FootstepPlanningRequestPacket otherMyClass = (FootstepPlanningRequestPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.initial_stance_robot_side_ != otherMyClass.initial_stance_robot_side_) return false;

      if (!this.stance_foot_position_in_world_.equals(otherMyClass.stance_foot_position_in_world_)) return false;
      if (!this.stance_foot_orientation_in_world_.equals(otherMyClass.stance_foot_orientation_in_world_)) return false;
      if (!this.goal_position_in_world_.equals(otherMyClass.goal_position_in_world_)) return false;
      if (!this.goal_orientation_in_world_.equals(otherMyClass.goal_orientation_in_world_)) return false;
      if(this.requested_footstep_planner_type_ != otherMyClass.requested_footstep_planner_type_) return false;

      if(this.timeout_ != otherMyClass.timeout_) return false;

      if(this.horizon_length_ != otherMyClass.horizon_length_) return false;

      if (!this.planar_regions_list_message_.equals(otherMyClass.planar_regions_list_message_)) return false;
      if(this.assume_flat_ground_ != otherMyClass.assume_flat_ground_) return false;

      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanningRequestPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("initial_stance_robot_side=");
      builder.append(this.initial_stance_robot_side_);      builder.append(", ");
      builder.append("stance_foot_position_in_world=");
      builder.append(this.stance_foot_position_in_world_);      builder.append(", ");
      builder.append("stance_foot_orientation_in_world=");
      builder.append(this.stance_foot_orientation_in_world_);      builder.append(", ");
      builder.append("goal_position_in_world=");
      builder.append(this.goal_position_in_world_);      builder.append(", ");
      builder.append("goal_orientation_in_world=");
      builder.append(this.goal_orientation_in_world_);      builder.append(", ");
      builder.append("requested_footstep_planner_type=");
      builder.append(this.requested_footstep_planner_type_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("horizon_length=");
      builder.append(this.horizon_length_);      builder.append(", ");
      builder.append("planar_regions_list_message=");
      builder.append(this.planar_regions_list_message_);      builder.append(", ");
      builder.append("assume_flat_ground=");
      builder.append(this.assume_flat_ground_);      builder.append(", ");
      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);
      builder.append("}");
      return builder.toString();
   }
}
