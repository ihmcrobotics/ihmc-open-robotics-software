package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       */
public class MultiContactTrajectoryStatus extends Packet<MultiContactTrajectoryStatus> implements Settable<MultiContactTrajectoryStatus>, EpsilonComparable<MultiContactTrajectoryStatus>
{
   public static final byte TRAJECTORY_STATUS_STARTED = (byte) 0;
   public static final byte TRAJECTORY_STATUS_COMPLETED = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Reports the status of the trajectory
            */
   public byte trajectory_status_ = (byte) 255;

   public MultiContactTrajectoryStatus()
   {
   }

   public MultiContactTrajectoryStatus(MultiContactTrajectoryStatus other)
   {
      this();
      set(other);
   }

   public void set(MultiContactTrajectoryStatus other)
   {
      sequence_id_ = other.sequence_id_;

      trajectory_status_ = other.trajectory_status_;

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
            * Reports the status of the trajectory
            */
   public void setTrajectoryStatus(byte trajectory_status)
   {
      trajectory_status_ = trajectory_status;
   }
   /**
            * Reports the status of the trajectory
            */
   public byte getTrajectoryStatus()
   {
      return trajectory_status_;
   }


   public static Supplier<MultiContactTrajectoryStatusPubSubType> getPubSubType()
   {
      return MultiContactTrajectoryStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactTrajectoryStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactTrajectoryStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_status_, other.trajectory_status_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactTrajectoryStatus)) return false;

      MultiContactTrajectoryStatus otherMyClass = (MultiContactTrajectoryStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.trajectory_status_ != otherMyClass.trajectory_status_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactTrajectoryStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("trajectory_status=");
      builder.append(this.trajectory_status_);
      builder.append("}");
      return builder.toString();
   }
}
