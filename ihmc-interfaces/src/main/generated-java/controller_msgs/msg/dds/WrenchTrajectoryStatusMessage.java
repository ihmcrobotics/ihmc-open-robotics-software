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
public class WrenchTrajectoryStatusMessage extends Packet<WrenchTrajectoryStatusMessage> implements Settable<WrenchTrajectoryStatusMessage>, EpsilonComparable<WrenchTrajectoryStatusMessage>
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
            * Refers to the name of the end-effector that executed the trajectory.
            */
   public java.lang.StringBuilder end_effector_name_;
   /**
            * The current status for the trajectory execution.
            */
   public byte trajectory_execution_status_ = (byte) 255;
   /**
            * Time in trajectory in seconds.
            */
   public double timestamp_;

   public WrenchTrajectoryStatusMessage()
   {
      end_effector_name_ = new java.lang.StringBuilder(255);
   }

   public WrenchTrajectoryStatusMessage(WrenchTrajectoryStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(WrenchTrajectoryStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      end_effector_name_.setLength(0);
      end_effector_name_.append(other.end_effector_name_);

      trajectory_execution_status_ = other.trajectory_execution_status_;

      timestamp_ = other.timestamp_;

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
            * Refers to the name of the end-effector that executed the trajectory.
            */
   public void setEndEffectorName(java.lang.String end_effector_name)
   {
      end_effector_name_.setLength(0);
      end_effector_name_.append(end_effector_name);
   }

   /**
            * Refers to the name of the end-effector that executed the trajectory.
            */
   public java.lang.String getEndEffectorNameAsString()
   {
      return getEndEffectorName().toString();
   }
   /**
            * Refers to the name of the end-effector that executed the trajectory.
            */
   public java.lang.StringBuilder getEndEffectorName()
   {
      return end_effector_name_;
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


   public static Supplier<WrenchTrajectoryStatusMessagePubSubType> getPubSubType()
   {
      return WrenchTrajectoryStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WrenchTrajectoryStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WrenchTrajectoryStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.end_effector_name_, other.end_effector_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_execution_status_, other.trajectory_execution_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WrenchTrajectoryStatusMessage)) return false;

      WrenchTrajectoryStatusMessage otherMyClass = (WrenchTrajectoryStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.end_effector_name_, otherMyClass.end_effector_name_)) return false;

      if(this.trajectory_execution_status_ != otherMyClass.trajectory_execution_status_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WrenchTrajectoryStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("end_effector_name=");
      builder.append(this.end_effector_name_);      builder.append(", ");
      builder.append("trajectory_execution_status=");
      builder.append(this.trajectory_execution_status_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);
      builder.append("}");
      return builder.toString();
   }
}
