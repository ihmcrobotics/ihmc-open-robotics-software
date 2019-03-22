package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * The walking controller uses this message to report when it completed the execution of a trajectory message.
       */
public class TaskspaceTrajectoryStatusMessage extends Packet<TaskspaceTrajectoryStatusMessage> implements Settable<TaskspaceTrajectoryStatusMessage>, EpsilonComparable<TaskspaceTrajectoryStatusMessage>
{
   public static final byte TRAJECTORY_EXECUTION_STATUS_STARTED = (byte) 0;
   public static final byte TRAJECTORY_EXECUTION_STATUS_COMPLETED = (byte) 1;
   /**
            * Unique ID of the trajectory message this status refers to.
            */
   public long sequence_id_;
   /**
            * Refers to the name of the rigid-body that executed the trajectory.
            */
   public java.lang.StringBuilder body_name_;
   /**
            * The current status for the trajectory execution.
            */
   public byte trajectory_status_ = (byte) 255;
   /**
            * The desired position (in world coordinates) of the rigid-body sent to the controller.
            */
   public us.ihmc.euclid.tuple3D.Point3D desired_body_position_;
   /**
            * The desired orientation (in world coordinates) of the rigid-body sent to the controller.
            */
   public us.ihmc.euclid.tuple4D.Quaternion desired_foot_orientation_;
   /**
            * The position (in world coordinates) of where the rigid-body actually arrived when the trajectory finished.
            */
   public us.ihmc.euclid.tuple3D.Point3D actual_body_position_;
   /**
            * The orientation (in world coordinates) of where the rigid-body actually arrived when the trajectory finished.
            */
   public us.ihmc.euclid.tuple4D.Quaternion actual_body_orientation_;

   public TaskspaceTrajectoryStatusMessage()
   {
      body_name_ = new java.lang.StringBuilder(255);
      desired_body_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      desired_foot_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      actual_body_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      actual_body_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public TaskspaceTrajectoryStatusMessage(TaskspaceTrajectoryStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(TaskspaceTrajectoryStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      body_name_.setLength(0);
      body_name_.append(other.body_name_);

      trajectory_status_ = other.trajectory_status_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_body_position_, desired_body_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_foot_orientation_, desired_foot_orientation_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.actual_body_position_, actual_body_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.actual_body_orientation_, actual_body_orientation_);
   }

   /**
            * Unique ID of the trajectory message this status refers to.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID of the trajectory message this status refers to.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Refers to the name of the rigid-body that executed the trajectory.
            */
   public void setBodyName(java.lang.String body_name)
   {
      body_name_.setLength(0);
      body_name_.append(body_name);
   }

   /**
            * Refers to the name of the rigid-body that executed the trajectory.
            */
   public java.lang.String getBodyNameAsString()
   {
      return getBodyName().toString();
   }
   /**
            * Refers to the name of the rigid-body that executed the trajectory.
            */
   public java.lang.StringBuilder getBodyName()
   {
      return body_name_;
   }

   /**
            * The current status for the trajectory execution.
            */
   public void setTrajectoryStatus(byte trajectory_status)
   {
      trajectory_status_ = trajectory_status;
   }
   /**
            * The current status for the trajectory execution.
            */
   public byte getTrajectoryStatus()
   {
      return trajectory_status_;
   }


   /**
            * The desired position (in world coordinates) of the rigid-body sent to the controller.
            */
   public us.ihmc.euclid.tuple3D.Point3D getDesiredBodyPosition()
   {
      return desired_body_position_;
   }


   /**
            * The desired orientation (in world coordinates) of the rigid-body sent to the controller.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getDesiredFootOrientation()
   {
      return desired_foot_orientation_;
   }


   /**
            * The position (in world coordinates) of where the rigid-body actually arrived when the trajectory finished.
            */
   public us.ihmc.euclid.tuple3D.Point3D getActualBodyPosition()
   {
      return actual_body_position_;
   }


   /**
            * The orientation (in world coordinates) of where the rigid-body actually arrived when the trajectory finished.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getActualBodyOrientation()
   {
      return actual_body_orientation_;
   }


   public static Supplier<TaskspaceTrajectoryStatusMessagePubSubType> getPubSubType()
   {
      return TaskspaceTrajectoryStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TaskspaceTrajectoryStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TaskspaceTrajectoryStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.body_name_, other.body_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_status_, other.trajectory_status_, epsilon)) return false;

      if (!this.desired_body_position_.epsilonEquals(other.desired_body_position_, epsilon)) return false;
      if (!this.desired_foot_orientation_.epsilonEquals(other.desired_foot_orientation_, epsilon)) return false;
      if (!this.actual_body_position_.epsilonEquals(other.actual_body_position_, epsilon)) return false;
      if (!this.actual_body_orientation_.epsilonEquals(other.actual_body_orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TaskspaceTrajectoryStatusMessage)) return false;

      TaskspaceTrajectoryStatusMessage otherMyClass = (TaskspaceTrajectoryStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.body_name_, otherMyClass.body_name_)) return false;

      if(this.trajectory_status_ != otherMyClass.trajectory_status_) return false;

      if (!this.desired_body_position_.equals(otherMyClass.desired_body_position_)) return false;
      if (!this.desired_foot_orientation_.equals(otherMyClass.desired_foot_orientation_)) return false;
      if (!this.actual_body_position_.equals(otherMyClass.actual_body_position_)) return false;
      if (!this.actual_body_orientation_.equals(otherMyClass.actual_body_orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TaskspaceTrajectoryStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("body_name=");
      builder.append(this.body_name_);      builder.append(", ");
      builder.append("trajectory_status=");
      builder.append(this.trajectory_status_);      builder.append(", ");
      builder.append("desired_body_position=");
      builder.append(this.desired_body_position_);      builder.append(", ");
      builder.append("desired_foot_orientation=");
      builder.append(this.desired_foot_orientation_);      builder.append(", ");
      builder.append("actual_body_position=");
      builder.append(this.actual_body_position_);      builder.append(", ");
      builder.append("actual_body_orientation=");
      builder.append(this.actual_body_orientation_);
      builder.append("}");
      return builder.toString();
   }
}
