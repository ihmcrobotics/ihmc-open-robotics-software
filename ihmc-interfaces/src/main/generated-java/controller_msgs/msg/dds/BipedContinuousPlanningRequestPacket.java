package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped footstep planning module.
       */
public class BipedContinuousPlanningRequestPacket extends Packet<BipedContinuousPlanningRequestPacket> implements Settable<BipedContinuousPlanningRequestPacket>, EpsilonComparable<BipedContinuousPlanningRequestPacket>
{

   public static final byte FOOTSTEP_PLANNER_TARGET_TYPE_POSE_BETWEEN_FEET = (byte) 0;

   public static final byte FOOTSTEP_PLANNER_TARGET_TYPE_FOOTSTEPS = (byte) 1;

   public static final int UNSET_ID = -1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte start_target_type_;

   public us.ihmc.euclid.tuple3D.Point3D left_start_position_in_world_;

   public us.ihmc.euclid.tuple4D.Quaternion left_start_orientation_in_world_;

   public us.ihmc.euclid.tuple3D.Point3D right_start_position_in_world_;

   public us.ihmc.euclid.tuple4D.Quaternion right_start_orientation_in_world_;

   public us.ihmc.euclid.tuple3D.Point3D goal_position_in_world_;

   public us.ihmc.euclid.tuple4D.Quaternion goal_orientation_in_world_;

   public int planner_request_id_ = -1;

   public double timeout_;

   public double best_effort_timeout_;

   public int max_iterations_ = -1;

   public double horizon_length_;

   public BipedContinuousPlanningRequestPacket()
   {



      left_start_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();

      left_start_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();

      right_start_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();

      right_start_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();

      goal_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();

      goal_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();






   }

   public BipedContinuousPlanningRequestPacket(BipedContinuousPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(BipedContinuousPlanningRequestPacket other)
   {

      sequence_id_ = other.sequence_id_;


      start_target_type_ = other.start_target_type_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.left_start_position_in_world_, left_start_position_in_world_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.left_start_orientation_in_world_, left_start_orientation_in_world_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.right_start_position_in_world_, right_start_position_in_world_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.right_start_orientation_in_world_, right_start_orientation_in_world_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_position_in_world_, goal_position_in_world_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.goal_orientation_in_world_, goal_orientation_in_world_);

      planner_request_id_ = other.planner_request_id_;


      timeout_ = other.timeout_;


      best_effort_timeout_ = other.best_effort_timeout_;


      max_iterations_ = other.max_iterations_;


      horizon_length_ = other.horizon_length_;

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



   public us.ihmc.euclid.tuple3D.Point3D getLeftStartPositionInWorld()
   {
      return left_start_position_in_world_;
   }



   public us.ihmc.euclid.tuple4D.Quaternion getLeftStartOrientationInWorld()
   {
      return left_start_orientation_in_world_;
   }



   public us.ihmc.euclid.tuple3D.Point3D getRightStartPositionInWorld()
   {
      return right_start_position_in_world_;
   }



   public us.ihmc.euclid.tuple4D.Quaternion getRightStartOrientationInWorld()
   {
      return right_start_orientation_in_world_;
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


   public void setBestEffortTimeout(double best_effort_timeout)
   {
      best_effort_timeout_ = best_effort_timeout;
   }
   public double getBestEffortTimeout()
   {
      return best_effort_timeout_;
   }


   public void setMaxIterations(int max_iterations)
   {
      max_iterations_ = max_iterations;
   }
   public int getMaxIterations()
   {
      return max_iterations_;
   }


   public void setHorizonLength(double horizon_length)
   {
      horizon_length_ = horizon_length;
   }
   public double getHorizonLength()
   {
      return horizon_length_;
   }


   public static Supplier<BipedContinuousPlanningRequestPacketPubSubType> getPubSubType()
   {
      return BipedContinuousPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BipedContinuousPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BipedContinuousPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_target_type_, other.start_target_type_, epsilon)) return false;


      if (!this.left_start_position_in_world_.epsilonEquals(other.left_start_position_in_world_, epsilon)) return false;

      if (!this.left_start_orientation_in_world_.epsilonEquals(other.left_start_orientation_in_world_, epsilon)) return false;

      if (!this.right_start_position_in_world_.epsilonEquals(other.right_start_position_in_world_, epsilon)) return false;

      if (!this.right_start_orientation_in_world_.epsilonEquals(other.right_start_orientation_in_world_, epsilon)) return false;

      if (!this.goal_position_in_world_.epsilonEquals(other.goal_position_in_world_, epsilon)) return false;

      if (!this.goal_orientation_in_world_.epsilonEquals(other.goal_orientation_in_world_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.best_effort_timeout_, other.best_effort_timeout_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_iterations_, other.max_iterations_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.horizon_length_, other.horizon_length_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BipedContinuousPlanningRequestPacket)) return false;

      BipedContinuousPlanningRequestPacket otherMyClass = (BipedContinuousPlanningRequestPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.start_target_type_ != otherMyClass.start_target_type_) return false;


      if (!this.left_start_position_in_world_.equals(otherMyClass.left_start_position_in_world_)) return false;

      if (!this.left_start_orientation_in_world_.equals(otherMyClass.left_start_orientation_in_world_)) return false;

      if (!this.right_start_position_in_world_.equals(otherMyClass.right_start_position_in_world_)) return false;

      if (!this.right_start_orientation_in_world_.equals(otherMyClass.right_start_orientation_in_world_)) return false;

      if (!this.goal_position_in_world_.equals(otherMyClass.goal_position_in_world_)) return false;

      if (!this.goal_orientation_in_world_.equals(otherMyClass.goal_orientation_in_world_)) return false;

      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;


      if(this.timeout_ != otherMyClass.timeout_) return false;


      if(this.best_effort_timeout_ != otherMyClass.best_effort_timeout_) return false;


      if(this.max_iterations_ != otherMyClass.max_iterations_) return false;


      if(this.horizon_length_ != otherMyClass.horizon_length_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BipedContinuousPlanningRequestPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("start_target_type=");
      builder.append(this.start_target_type_);      builder.append(", ");

      builder.append("left_start_position_in_world=");
      builder.append(this.left_start_position_in_world_);      builder.append(", ");

      builder.append("left_start_orientation_in_world=");
      builder.append(this.left_start_orientation_in_world_);      builder.append(", ");

      builder.append("right_start_position_in_world=");
      builder.append(this.right_start_position_in_world_);      builder.append(", ");

      builder.append("right_start_orientation_in_world=");
      builder.append(this.right_start_orientation_in_world_);      builder.append(", ");

      builder.append("goal_position_in_world=");
      builder.append(this.goal_position_in_world_);      builder.append(", ");

      builder.append("goal_orientation_in_world=");
      builder.append(this.goal_orientation_in_world_);      builder.append(", ");

      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);      builder.append(", ");

      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");

      builder.append("best_effort_timeout=");
      builder.append(this.best_effort_timeout_);      builder.append(", ");

      builder.append("max_iterations=");
      builder.append(this.max_iterations_);      builder.append(", ");

      builder.append("horizon_length=");
      builder.append(this.horizon_length_);
      builder.append("}");
      return builder.toString();
   }
}
