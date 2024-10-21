package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This class is used to change the PID gains over time.
       */
public class SE3PIDGainsTrajectoryMessage extends Packet<SE3PIDGainsTrajectoryMessage> implements Settable<SE3PIDGainsTrajectoryMessage>, EpsilonComparable<SE3PIDGainsTrajectoryMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * List of trajectory points (in taskpsace) to go through while executing the trajectory.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage>  pid_gains_trajectory_points_;
   /**
            * Properties for queueing trajectories.
            */
   public ihmc_common_msgs.msg.dds.QueueableMessage queueing_properties_;

   public SE3PIDGainsTrajectoryMessage()
   {
      pid_gains_trajectory_points_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage> (200, new controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessagePubSubType());
      queueing_properties_ = new ihmc_common_msgs.msg.dds.QueueableMessage();

   }

   public SE3PIDGainsTrajectoryMessage(SE3PIDGainsTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(SE3PIDGainsTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      pid_gains_trajectory_points_.set(other.pid_gains_trajectory_points_);
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
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
            * List of trajectory points (in taskpsace) to go through while executing the trajectory.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage>  getPidGainsTrajectoryPoints()
   {
      return pid_gains_trajectory_points_;
   }


   /**
            * Properties for queueing trajectories.
            */
   public ihmc_common_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   public static Supplier<SE3PIDGainsTrajectoryMessagePubSubType> getPubSubType()
   {
      return SE3PIDGainsTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SE3PIDGainsTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SE3PIDGainsTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.pid_gains_trajectory_points_.size() != other.pid_gains_trajectory_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.pid_gains_trajectory_points_.size(); i++)
         {  if (!this.pid_gains_trajectory_points_.get(i).epsilonEquals(other.pid_gains_trajectory_points_.get(i), epsilon)) return false; }
      }

      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SE3PIDGainsTrajectoryMessage)) return false;

      SE3PIDGainsTrajectoryMessage otherMyClass = (SE3PIDGainsTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.pid_gains_trajectory_points_.equals(otherMyClass.pid_gains_trajectory_points_)) return false;
      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SE3PIDGainsTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("pid_gains_trajectory_points=");
      builder.append(this.pid_gains_trajectory_points_);      builder.append(", ");
      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);
      builder.append("}");
      return builder.toString();
   }
}
