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
public class JointspaceTrajectoryStatusMessage extends Packet<JointspaceTrajectoryStatusMessage> implements Settable<JointspaceTrajectoryStatusMessage>, EpsilonComparable<JointspaceTrajectoryStatusMessage>
{
   /**
          * The trajectory input was received and accepted, the execution just started.
          */
   public static final byte TRAJECTORY_EXECUTION_STATUS_STARTED = (byte) 0;
   /**
          * The trajectory input was received, accepted, and its execution just finished.
          */
   public static final byte TRAJECTORY_EXECUTION_STATUS_COMPLETED = (byte) 1;
   /**
            * Unique ID of the trajectory message this status refers to.
            */
   public long sequence_id_;
   /**
            * Refers to the names of the joints that executed the trajectory.
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  joint_names_;
   /**
            * The current status for the trajectory execution.
            */
   public byte trajectory_execution_status_ = (byte) 255;
   /**
            * Time in trajectory in seconds.
            */
   public double timestamp_;
   /**
            * The desired positions of the joints as sent to the controller.
            */
   public us.ihmc.idl.IDLSequence.Double  desired_joint_positions_;
   /**
            * The positions of where the joints actually arrived when the trajectory finished.
            */
   public us.ihmc.idl.IDLSequence.Double  actual_joint_positions_;

   public JointspaceTrajectoryStatusMessage()
   {
      joint_names_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      desired_joint_positions_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

      actual_joint_positions_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

   }

   public JointspaceTrajectoryStatusMessage(JointspaceTrajectoryStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(JointspaceTrajectoryStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_names_.set(other.joint_names_);
      trajectory_execution_status_ = other.trajectory_execution_status_;

      timestamp_ = other.timestamp_;

      desired_joint_positions_.set(other.desired_joint_positions_);
      actual_joint_positions_.set(other.actual_joint_positions_);
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
            * Refers to the names of the joints that executed the trajectory.
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getJointNames()
   {
      return joint_names_;
   }

   /**
            * The current status for the trajectory execution.
            */
   public void setTrajectoryExecutionStatus(byte trajectory_execution_status)
   {
      trajectory_execution_status_ = trajectory_execution_status;
   }
   /**
            * The current status for the trajectory execution.
            */
   public byte getTrajectoryExecutionStatus()
   {
      return trajectory_execution_status_;
   }

   /**
            * Time in trajectory in seconds.
            */
   public void setTimestamp(double timestamp)
   {
      timestamp_ = timestamp;
   }
   /**
            * Time in trajectory in seconds.
            */
   public double getTimestamp()
   {
      return timestamp_;
   }


   /**
            * The desired positions of the joints as sent to the controller.
            */
   public us.ihmc.idl.IDLSequence.Double  getDesiredJointPositions()
   {
      return desired_joint_positions_;
   }


   /**
            * The positions of where the joints actually arrived when the trajectory finished.
            */
   public us.ihmc.idl.IDLSequence.Double  getActualJointPositions()
   {
      return actual_joint_positions_;
   }


   public static Supplier<JointspaceTrajectoryStatusMessagePubSubType> getPubSubType()
   {
      return JointspaceTrajectoryStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointspaceTrajectoryStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointspaceTrajectoryStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.joint_names_, other.joint_names_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_execution_status_, other.trajectory_execution_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.desired_joint_positions_, other.desired_joint_positions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.actual_joint_positions_, other.actual_joint_positions_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointspaceTrajectoryStatusMessage)) return false;

      JointspaceTrajectoryStatusMessage otherMyClass = (JointspaceTrajectoryStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.joint_names_.equals(otherMyClass.joint_names_)) return false;
      if(this.trajectory_execution_status_ != otherMyClass.trajectory_execution_status_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if (!this.desired_joint_positions_.equals(otherMyClass.desired_joint_positions_)) return false;
      if (!this.actual_joint_positions_.equals(otherMyClass.actual_joint_positions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointspaceTrajectoryStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("joint_names=");
      builder.append(this.joint_names_);      builder.append(", ");
      builder.append("trajectory_execution_status=");
      builder.append(this.trajectory_execution_status_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("desired_joint_positions=");
      builder.append(this.desired_joint_positions_);      builder.append(", ");
      builder.append("actual_joint_positions=");
      builder.append(this.actual_joint_positions_);
      builder.append("}");
      return builder.toString();
   }
}
