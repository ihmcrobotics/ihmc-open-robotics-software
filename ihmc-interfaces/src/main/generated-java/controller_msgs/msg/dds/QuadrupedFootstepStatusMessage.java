package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message gives the status of the current footstep from the quadruped controller as well as the position of the footstep in world coordinates.
       */
public class QuadrupedFootstepStatusMessage extends Packet<QuadrupedFootstepStatusMessage> implements Settable<QuadrupedFootstepStatusMessage>, EpsilonComparable<QuadrupedFootstepStatusMessage>
{
   public static final byte FOOTSTEP_STATUS_STARTED = (byte) 0;
   public static final byte FOOTSTEP_STATUS_COMPLETED = (byte) 1;
   public static final byte FRONT_LEFT = (byte) 0;
   public static final byte FRONT_RIGHT = (byte) 1;
   public static final byte HIND_RIGHT = (byte) 2;
   public static final byte HIND_LEFT = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The current footstep status enum value.
            */
   public byte footstep_status_ = (byte) 255;
   /**
            * footstep_index starts at 0 and monotonically increases during a controller session
            */
   public int footstep_index_;
   /**
            * The robot quadrant that this footstep status correlates to.
            */
   public byte robot_quadrant_ = (byte) 255;
   /**
            * Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public us.ihmc.euclid.tuple3D.Point3D desired_touchdown_position_in_world_;
   /**
            * Specifies the position of where the foot actually landed, only valid when the status is FOOTSTEP_STATUS_COMPLETED
            */
   public us.ihmc.euclid.tuple3D.Point3D actual_touchdown_position_in_world_;
   /**
            * Specifies the desired timing of the step
            */
   public controller_msgs.msg.dds.TimeIntervalMessage desired_step_interval_;
   /**
            * Specifies the achieved timing of the step, the end time is only valid when the status is FOOTSTEP_STATUS_COMPLETED
            */
   public controller_msgs.msg.dds.TimeIntervalMessage actual_step_interval_;

   public QuadrupedFootstepStatusMessage()
   {
      desired_touchdown_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      actual_touchdown_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      desired_step_interval_ = new controller_msgs.msg.dds.TimeIntervalMessage();
      actual_step_interval_ = new controller_msgs.msg.dds.TimeIntervalMessage();
   }

   public QuadrupedFootstepStatusMessage(QuadrupedFootstepStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedFootstepStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      footstep_status_ = other.footstep_status_;

      footstep_index_ = other.footstep_index_;

      robot_quadrant_ = other.robot_quadrant_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_touchdown_position_in_world_, desired_touchdown_position_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.actual_touchdown_position_in_world_, actual_touchdown_position_in_world_);
      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.staticCopy(other.desired_step_interval_, desired_step_interval_);
      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.staticCopy(other.actual_step_interval_, actual_step_interval_);
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
            * The current footstep status enum value.
            */
   public void setFootstepStatus(byte footstep_status)
   {
      footstep_status_ = footstep_status;
   }
   /**
            * The current footstep status enum value.
            */
   public byte getFootstepStatus()
   {
      return footstep_status_;
   }

   /**
            * footstep_index starts at 0 and monotonically increases during a controller session
            */
   public void setFootstepIndex(int footstep_index)
   {
      footstep_index_ = footstep_index;
   }
   /**
            * footstep_index starts at 0 and monotonically increases during a controller session
            */
   public int getFootstepIndex()
   {
      return footstep_index_;
   }

   /**
            * The robot quadrant that this footstep status correlates to.
            */
   public void setRobotQuadrant(byte robot_quadrant)
   {
      robot_quadrant_ = robot_quadrant;
   }
   /**
            * The robot quadrant that this footstep status correlates to.
            */
   public byte getRobotQuadrant()
   {
      return robot_quadrant_;
   }


   /**
            * Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public us.ihmc.euclid.tuple3D.Point3D getDesiredTouchdownPositionInWorld()
   {
      return desired_touchdown_position_in_world_;
   }


   /**
            * Specifies the position of where the foot actually landed, only valid when the status is FOOTSTEP_STATUS_COMPLETED
            */
   public us.ihmc.euclid.tuple3D.Point3D getActualTouchdownPositionInWorld()
   {
      return actual_touchdown_position_in_world_;
   }


   /**
            * Specifies the desired timing of the step
            */
   public controller_msgs.msg.dds.TimeIntervalMessage getDesiredStepInterval()
   {
      return desired_step_interval_;
   }


   /**
            * Specifies the achieved timing of the step, the end time is only valid when the status is FOOTSTEP_STATUS_COMPLETED
            */
   public controller_msgs.msg.dds.TimeIntervalMessage getActualStepInterval()
   {
      return actual_step_interval_;
   }


   public static Supplier<QuadrupedFootstepStatusMessagePubSubType> getPubSubType()
   {
      return QuadrupedFootstepStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedFootstepStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedFootstepStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_status_, other.footstep_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_index_, other.footstep_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_quadrant_, other.robot_quadrant_, epsilon)) return false;

      if (!this.desired_touchdown_position_in_world_.epsilonEquals(other.desired_touchdown_position_in_world_, epsilon)) return false;
      if (!this.actual_touchdown_position_in_world_.epsilonEquals(other.actual_touchdown_position_in_world_, epsilon)) return false;
      if (!this.desired_step_interval_.epsilonEquals(other.desired_step_interval_, epsilon)) return false;
      if (!this.actual_step_interval_.epsilonEquals(other.actual_step_interval_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedFootstepStatusMessage)) return false;

      QuadrupedFootstepStatusMessage otherMyClass = (QuadrupedFootstepStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.footstep_status_ != otherMyClass.footstep_status_) return false;

      if(this.footstep_index_ != otherMyClass.footstep_index_) return false;

      if(this.robot_quadrant_ != otherMyClass.robot_quadrant_) return false;

      if (!this.desired_touchdown_position_in_world_.equals(otherMyClass.desired_touchdown_position_in_world_)) return false;
      if (!this.actual_touchdown_position_in_world_.equals(otherMyClass.actual_touchdown_position_in_world_)) return false;
      if (!this.desired_step_interval_.equals(otherMyClass.desired_step_interval_)) return false;
      if (!this.actual_step_interval_.equals(otherMyClass.actual_step_interval_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedFootstepStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("footstep_status=");
      builder.append(this.footstep_status_);      builder.append(", ");
      builder.append("footstep_index=");
      builder.append(this.footstep_index_);      builder.append(", ");
      builder.append("robot_quadrant=");
      builder.append(this.robot_quadrant_);      builder.append(", ");
      builder.append("desired_touchdown_position_in_world=");
      builder.append(this.desired_touchdown_position_in_world_);      builder.append(", ");
      builder.append("actual_touchdown_position_in_world=");
      builder.append(this.actual_touchdown_position_in_world_);      builder.append(", ");
      builder.append("desired_step_interval=");
      builder.append(this.desired_step_interval_);      builder.append(", ");
      builder.append("actual_step_interval=");
      builder.append(this.actual_step_interval_);
      builder.append("}");
      return builder.toString();
   }
}
