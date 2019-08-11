package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped footstep planning module.
       */
public class QuadrupedContinuousPlanningRequestPacket extends Packet<QuadrupedContinuousPlanningRequestPacket> implements Settable<QuadrupedContinuousPlanningRequestPacket>, EpsilonComparable<QuadrupedContinuousPlanningRequestPacket>
{
   public static final byte FOOTSTEP_PLANNER_TARGET_TYPE_POSE_BETWEEN_FEET = (byte) 0;
   public static final byte FOOTSTEP_PLANNER_TARGET_TYPE_FOOTSTEPS = (byte) 1;
   public static final int UNSET_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte start_target_type_;
   public us.ihmc.euclid.tuple3D.Point3D goal_position_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion goal_orientation_in_world_;
   public int planner_request_id_ = -1;
   public double timeout_;

   public QuadrupedContinuousPlanningRequestPacket()
   {
      goal_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public QuadrupedContinuousPlanningRequestPacket(QuadrupedContinuousPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedContinuousPlanningRequestPacket other)
   {
      sequence_id_ = other.sequence_id_;

      start_target_type_ = other.start_target_type_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_position_in_world_, goal_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.goal_orientation_in_world_, goal_orientation_in_world_);
      planner_request_id_ = other.planner_request_id_;

      timeout_ = other.timeout_;

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

   public void setStartTargetType(byte start_target_type)
   {
      start_target_type_ = start_target_type;
   }
   public byte getStartTargetType()
   {
      return start_target_type_;
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

   public void setTimeout(double timeout)
   {
      timeout_ = timeout;
   }
   public double getTimeout()
   {
      return timeout_;
   }


   public static Supplier<QuadrupedContinuousPlanningRequestPacketPubSubType> getPubSubType()
   {
      return QuadrupedContinuousPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedContinuousPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedContinuousPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_target_type_, other.start_target_type_, epsilon)) return false;

      if (!this.goal_position_in_world_.epsilonEquals(other.goal_position_in_world_, epsilon)) return false;
      if (!this.goal_orientation_in_world_.epsilonEquals(other.goal_orientation_in_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedContinuousPlanningRequestPacket)) return false;

      QuadrupedContinuousPlanningRequestPacket otherMyClass = (QuadrupedContinuousPlanningRequestPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.start_target_type_ != otherMyClass.start_target_type_) return false;

      if (!this.goal_position_in_world_.equals(otherMyClass.goal_position_in_world_)) return false;
      if (!this.goal_orientation_in_world_.equals(otherMyClass.goal_orientation_in_world_)) return false;
      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;

      if(this.timeout_ != otherMyClass.timeout_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedContinuousPlanningRequestPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("start_target_type=");
      builder.append(this.start_target_type_);      builder.append(", ");
      builder.append("goal_position_in_world=");
      builder.append(this.goal_position_in_world_);      builder.append(", ");
      builder.append("goal_orientation_in_world=");
      builder.append(this.goal_orientation_in_world_);      builder.append(", ");
      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);
      builder.append("}");
      return builder.toString();
   }
}
