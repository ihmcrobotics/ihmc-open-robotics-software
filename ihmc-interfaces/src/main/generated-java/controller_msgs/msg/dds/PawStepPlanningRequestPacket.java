package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped footstep planning module.
       */
public class PawStepPlanningRequestPacket extends Packet<PawStepPlanningRequestPacket> implements Settable<PawStepPlanningRequestPacket>, EpsilonComparable<PawStepPlanningRequestPacket>
{
   public static final byte ROBOT_QUADRANT_FRONT_LEFT = (byte) 0;
   public static final byte ROBOT_QUADRANT_FRONT_RIGHT = (byte) 1;
   public static final byte ROBOT_QUADRANT_HIND_LEFT = (byte) 2;
   public static final byte ROBOT_QUADRANT_HIND_RIGHT = (byte) 3;
   public static final byte PAW_PLANNER_TARGET_TYPE_POSE_BETWEEN_FEET = (byte) 0;
   public static final byte PAW_PLANNER_TARGET_TYPE_FOOTSTEPS = (byte) 1;
   public static final int NO_PLAN_ID = -1;
   public static final byte PAW_PLANNER_TYPE_SIMPLE_PATH_TURN_WALK_TURN = (byte) 0;
   public static final byte PAW_PLANNER_TYPE_VIS_GRAPH_WITH_TURN_WALK_TURN = (byte) 1;
   public static final byte PAW_PLANNER_TYPE_A_STAR = (byte) 2;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte initial_step_robot_quadrant_ = (byte) 255;
   public byte start_target_type_;
   public us.ihmc.euclid.tuple3D.Point3D body_position_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion body_orientation_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D front_left_position_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D front_right_position_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D hind_left_position_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D hind_right_position_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D goal_position_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion goal_orientation_in_world_;
   public int planner_request_id_ = -1;
   public byte requested_paw_planner_type_ = (byte) 255;
   public double timeout_;
   public double best_effort_timeout_;
   public double horizon_length_;
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_message_;
   public boolean assume_flat_ground_;

   public PawStepPlanningRequestPacket()
   {
      body_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      body_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      front_left_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      front_right_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      hind_left_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      hind_right_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      planar_regions_list_message_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();
   }

   public PawStepPlanningRequestPacket(PawStepPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(PawStepPlanningRequestPacket other)
   {
      sequence_id_ = other.sequence_id_;

      initial_step_robot_quadrant_ = other.initial_step_robot_quadrant_;

      start_target_type_ = other.start_target_type_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.body_position_in_world_, body_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.body_orientation_in_world_, body_orientation_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.front_left_position_in_world_, front_left_position_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.front_right_position_in_world_, front_right_position_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.hind_left_position_in_world_, hind_left_position_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.hind_right_position_in_world_, hind_right_position_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_position_in_world_, goal_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.goal_orientation_in_world_, goal_orientation_in_world_);
      planner_request_id_ = other.planner_request_id_;

      requested_paw_planner_type_ = other.requested_paw_planner_type_;

      timeout_ = other.timeout_;

      best_effort_timeout_ = other.best_effort_timeout_;

      horizon_length_ = other.horizon_length_;

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_message_, planar_regions_list_message_);
      assume_flat_ground_ = other.assume_flat_ground_;

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

   public void setInitialStepRobotQuadrant(byte initial_step_robot_quadrant)
   {
      initial_step_robot_quadrant_ = initial_step_robot_quadrant;
   }
   public byte getInitialStepRobotQuadrant()
   {
      return initial_step_robot_quadrant_;
   }

   public void setStartTargetType(byte start_target_type)
   {
      start_target_type_ = start_target_type;
   }
   public byte getStartTargetType()
   {
      return start_target_type_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getBodyPositionInWorld()
   {
      return body_position_in_world_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getBodyOrientationInWorld()
   {
      return body_orientation_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getFrontLeftPositionInWorld()
   {
      return front_left_position_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getFrontRightPositionInWorld()
   {
      return front_right_position_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getHindLeftPositionInWorld()
   {
      return hind_left_position_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getHindRightPositionInWorld()
   {
      return hind_right_position_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getGoalPositionInWorld()
   {
      return goal_position_in_world_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getGoalOrientationInWorld()
   {
      return goal_orientation_in_world_;
   }

   public void setPlannerRequestId(int planner_request_id)
   {
      planner_request_id_ = planner_request_id;
   }
   public int getPlannerRequestId()
   {
      return planner_request_id_;
   }

   public void setRequestedPawPlannerType(byte requested_paw_planner_type)
   {
      requested_paw_planner_type_ = requested_paw_planner_type;
   }
   public byte getRequestedPawPlannerType()
   {
      return requested_paw_planner_type_;
   }

   public void setTimeout(double timeout)
   {
      timeout_ = timeout;
   }
   public double getTimeout()
   {
      return timeout_;
   }

   public void setBestEffortTimeout(double best_effort_timeout)
   {
      best_effort_timeout_ = best_effort_timeout;
   }
   public double getBestEffortTimeout()
   {
      return best_effort_timeout_;
   }

   public void setHorizonLength(double horizon_length)
   {
      horizon_length_ = horizon_length;
   }
   public double getHorizonLength()
   {
      return horizon_length_;
   }


   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsListMessage()
   {
      return planar_regions_list_message_;
   }

   public void setAssumeFlatGround(boolean assume_flat_ground)
   {
      assume_flat_ground_ = assume_flat_ground;
   }
   public boolean getAssumeFlatGround()
   {
      return assume_flat_ground_;
   }


   public static Supplier<PawStepPlanningRequestPacketPubSubType> getPubSubType()
   {
      return PawStepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PawStepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PawStepPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_step_robot_quadrant_, other.initial_step_robot_quadrant_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_target_type_, other.start_target_type_, epsilon)) return false;

      if (!this.body_position_in_world_.epsilonEquals(other.body_position_in_world_, epsilon)) return false;
      if (!this.body_orientation_in_world_.epsilonEquals(other.body_orientation_in_world_, epsilon)) return false;
      if (!this.front_left_position_in_world_.epsilonEquals(other.front_left_position_in_world_, epsilon)) return false;
      if (!this.front_right_position_in_world_.epsilonEquals(other.front_right_position_in_world_, epsilon)) return false;
      if (!this.hind_left_position_in_world_.epsilonEquals(other.hind_left_position_in_world_, epsilon)) return false;
      if (!this.hind_right_position_in_world_.epsilonEquals(other.hind_right_position_in_world_, epsilon)) return false;
      if (!this.goal_position_in_world_.epsilonEquals(other.goal_position_in_world_, epsilon)) return false;
      if (!this.goal_orientation_in_world_.epsilonEquals(other.goal_orientation_in_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_paw_planner_type_, other.requested_paw_planner_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.best_effort_timeout_, other.best_effort_timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.horizon_length_, other.horizon_length_, epsilon)) return false;

      if (!this.planar_regions_list_message_.epsilonEquals(other.planar_regions_list_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.assume_flat_ground_, other.assume_flat_ground_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PawStepPlanningRequestPacket)) return false;

      PawStepPlanningRequestPacket otherMyClass = (PawStepPlanningRequestPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.initial_step_robot_quadrant_ != otherMyClass.initial_step_robot_quadrant_) return false;

      if(this.start_target_type_ != otherMyClass.start_target_type_) return false;

      if (!this.body_position_in_world_.equals(otherMyClass.body_position_in_world_)) return false;
      if (!this.body_orientation_in_world_.equals(otherMyClass.body_orientation_in_world_)) return false;
      if (!this.front_left_position_in_world_.equals(otherMyClass.front_left_position_in_world_)) return false;
      if (!this.front_right_position_in_world_.equals(otherMyClass.front_right_position_in_world_)) return false;
      if (!this.hind_left_position_in_world_.equals(otherMyClass.hind_left_position_in_world_)) return false;
      if (!this.hind_right_position_in_world_.equals(otherMyClass.hind_right_position_in_world_)) return false;
      if (!this.goal_position_in_world_.equals(otherMyClass.goal_position_in_world_)) return false;
      if (!this.goal_orientation_in_world_.equals(otherMyClass.goal_orientation_in_world_)) return false;
      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;

      if(this.requested_paw_planner_type_ != otherMyClass.requested_paw_planner_type_) return false;

      if(this.timeout_ != otherMyClass.timeout_) return false;

      if(this.best_effort_timeout_ != otherMyClass.best_effort_timeout_) return false;

      if(this.horizon_length_ != otherMyClass.horizon_length_) return false;

      if (!this.planar_regions_list_message_.equals(otherMyClass.planar_regions_list_message_)) return false;
      if(this.assume_flat_ground_ != otherMyClass.assume_flat_ground_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PawStepPlanningRequestPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("initial_step_robot_quadrant=");
      builder.append(this.initial_step_robot_quadrant_);      builder.append(", ");
      builder.append("start_target_type=");
      builder.append(this.start_target_type_);      builder.append(", ");
      builder.append("body_position_in_world=");
      builder.append(this.body_position_in_world_);      builder.append(", ");
      builder.append("body_orientation_in_world=");
      builder.append(this.body_orientation_in_world_);      builder.append(", ");
      builder.append("front_left_position_in_world=");
      builder.append(this.front_left_position_in_world_);      builder.append(", ");
      builder.append("front_right_position_in_world=");
      builder.append(this.front_right_position_in_world_);      builder.append(", ");
      builder.append("hind_left_position_in_world=");
      builder.append(this.hind_left_position_in_world_);      builder.append(", ");
      builder.append("hind_right_position_in_world=");
      builder.append(this.hind_right_position_in_world_);      builder.append(", ");
      builder.append("goal_position_in_world=");
      builder.append(this.goal_position_in_world_);      builder.append(", ");
      builder.append("goal_orientation_in_world=");
      builder.append(this.goal_orientation_in_world_);      builder.append(", ");
      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);      builder.append(", ");
      builder.append("requested_paw_planner_type=");
      builder.append(this.requested_paw_planner_type_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("best_effort_timeout=");
      builder.append(this.best_effort_timeout_);      builder.append(", ");
      builder.append("horizon_length=");
      builder.append(this.horizon_length_);      builder.append(", ");
      builder.append("planar_regions_list_message=");
      builder.append(this.planar_regions_list_message_);      builder.append(", ");
      builder.append("assume_flat_ground=");
      builder.append(this.assume_flat_ground_);
      builder.append("}");
      return builder.toString();
   }
}
