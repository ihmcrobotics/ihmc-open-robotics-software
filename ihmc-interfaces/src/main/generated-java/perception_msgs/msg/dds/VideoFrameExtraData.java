package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * THIS MESSAGE IS SENT OVER VIDEO STREAMS.
       * MAX SIZE OF THIS MESSAGE IS LIMITED (~881 bytes from a bit of testing).
       * BE CAUTIOUS WHEN ADDING MORE FIELDS.
       * If adding a field that only needs to be sent once,
       * consider putting it in SRTStreamStatus.msg instead.
       */
public class VideoFrameExtraData extends Packet<VideoFrameExtraData> implements Settable<VideoFrameExtraData>, EpsilonComparable<VideoFrameExtraData>
{
   /**
            * Frame sequence number
            */
   public long sequence_number_;
   /**
            * Frame acquisition time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage acquisition_time_;
   /**
            * Sensor pose at time of acquisition
            */
   public us.ihmc.euclid.geometry.Pose3D sensor_pose_;

   public VideoFrameExtraData()
   {
      acquisition_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      sensor_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public VideoFrameExtraData(VideoFrameExtraData other)
   {
      this();
      set(other);
   }

   public void set(VideoFrameExtraData other)
   {
      sequence_number_ = other.sequence_number_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.acquisition_time_, acquisition_time_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.sensor_pose_, sensor_pose_);
   }

   /**
            * Frame sequence number
            */
   public void setSequenceNumber(long sequence_number)
   {
      sequence_number_ = sequence_number;
   }
   /**
            * Frame sequence number
            */
   public long getSequenceNumber()
   {
      return sequence_number_;
   }


   /**
            * Frame acquisition time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getAcquisitionTime()
   {
      return acquisition_time_;
   }


   /**
            * Sensor pose at time of acquisition
            */
   public us.ihmc.euclid.geometry.Pose3D getSensorPose()
   {
      return sensor_pose_;
   }


   public static Supplier<VideoFrameExtraDataPubSubType> getPubSubType()
   {
      return VideoFrameExtraDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VideoFrameExtraDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VideoFrameExtraData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_number_, other.sequence_number_, epsilon)) return false;

      if (!this.acquisition_time_.epsilonEquals(other.acquisition_time_, epsilon)) return false;
      if (!this.sensor_pose_.epsilonEquals(other.sensor_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VideoFrameExtraData)) return false;

      VideoFrameExtraData otherMyClass = (VideoFrameExtraData) other;

      if(this.sequence_number_ != otherMyClass.sequence_number_) return false;

      if (!this.acquisition_time_.equals(otherMyClass.acquisition_time_)) return false;
      if (!this.sensor_pose_.equals(otherMyClass.sensor_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VideoFrameExtraData {");
      builder.append("sequence_number=");
      builder.append(this.sequence_number_);      builder.append(", ");
      builder.append("acquisition_time=");
      builder.append(this.acquisition_time_);      builder.append(", ");
      builder.append("sensor_pose=");
      builder.append(this.sensor_pose_);
      builder.append("}");
      return builder.toString();
   }
}
