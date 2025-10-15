package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC localization module.
       */
public class StampedOdometryPacket extends Packet<StampedOdometryPacket> implements Settable<StampedOdometryPacket>, EpsilonComparable<StampedOdometryPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.euclid.geometry.Pose3D pose_;
   public us.ihmc.euclid.tuple4D.Quaternion imu_orientation_;
   public long timestamp_;
   public double confidence_factor_;

   public StampedOdometryPacket()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
      imu_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public StampedOdometryPacket(StampedOdometryPacket other)
   {
      this();
      set(other);
   }

   public void set(StampedOdometryPacket other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.imu_orientation_, imu_orientation_);
      timestamp_ = other.timestamp_;

      confidence_factor_ = other.confidence_factor_;

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


   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getImuOrientation()
   {
      return imu_orientation_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setConfidenceFactor(double confidence_factor)
   {
      confidence_factor_ = confidence_factor;
   }
   public double getConfidenceFactor()
   {
      return confidence_factor_;
   }


   public static Supplier<StampedOdometryPacketPubSubType> getPubSubType()
   {
      return StampedOdometryPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StampedOdometryPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StampedOdometryPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;
      if (!this.imu_orientation_.epsilonEquals(other.imu_orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_factor_, other.confidence_factor_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StampedOdometryPacket)) return false;

      StampedOdometryPacket otherMyClass = (StampedOdometryPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.pose_.equals(otherMyClass.pose_)) return false;
      if (!this.imu_orientation_.equals(otherMyClass.imu_orientation_)) return false;
      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.confidence_factor_ != otherMyClass.confidence_factor_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StampedOdometryPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");
      builder.append("imu_orientation=");
      builder.append(this.imu_orientation_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("confidence_factor=");
      builder.append(this.confidence_factor_);
      builder.append("}");
      return builder.toString();
   }
}
