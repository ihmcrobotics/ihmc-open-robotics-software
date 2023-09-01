package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message for sending large colored point clouds,
       * discretized and compressed with LZ4,
       * and publishing partial scans at a higher rate to allow smaller message sizes
       */
public class FusedSensorHeadPointCloudMessage extends Packet<FusedSensorHeadPointCloudMessage> implements Settable<FusedSensorHeadPointCloudMessage>, EpsilonComparable<FusedSensorHeadPointCloudMessage>
{
   /**
            * Approximate sensor data aquisition time seconds since epoch
            * Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
            */
   public long aquisition_seconds_since_epoch_;
   /**
            * Approximate sensor data aquisition time additional nanoseconds
            */
   public long aquisition_additional_nanos_;
   /**
            * Total number of segments
            */
   public long number_of_segments_;
   /**
            * Number of points per segment
            */
   public int points_per_segment_;
   /**
            * Segment this data represents
            */
   public long segment_index_;
   /**
            * Compressed point cloud data
            * Sized specifically for our current primary sensor head streaming
            * 22220 * 4 * 4
            */
   public us.ihmc.idl.IDLSequence.Byte  scan_;

   public FusedSensorHeadPointCloudMessage()
   {
      scan_ = new us.ihmc.idl.IDLSequence.Byte (7000000, "type_9");

   }

   public FusedSensorHeadPointCloudMessage(FusedSensorHeadPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(FusedSensorHeadPointCloudMessage other)
   {
      aquisition_seconds_since_epoch_ = other.aquisition_seconds_since_epoch_;

      aquisition_additional_nanos_ = other.aquisition_additional_nanos_;

      number_of_segments_ = other.number_of_segments_;

      points_per_segment_ = other.points_per_segment_;

      segment_index_ = other.segment_index_;

      scan_.set(other.scan_);
   }

   /**
            * Approximate sensor data aquisition time seconds since epoch
            * Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
            */
   public void setAquisitionSecondsSinceEpoch(long aquisition_seconds_since_epoch)
   {
      aquisition_seconds_since_epoch_ = aquisition_seconds_since_epoch;
   }
   /**
            * Approximate sensor data aquisition time seconds since epoch
            * Nanoseconds since the epoch goes beyond what a long can hold so we use two values.
            */
   public long getAquisitionSecondsSinceEpoch()
   {
      return aquisition_seconds_since_epoch_;
   }

   /**
            * Approximate sensor data aquisition time additional nanoseconds
            */
   public void setAquisitionAdditionalNanos(long aquisition_additional_nanos)
   {
      aquisition_additional_nanos_ = aquisition_additional_nanos;
   }
   /**
            * Approximate sensor data aquisition time additional nanoseconds
            */
   public long getAquisitionAdditionalNanos()
   {
      return aquisition_additional_nanos_;
   }

   /**
            * Total number of segments
            */
   public void setNumberOfSegments(long number_of_segments)
   {
      number_of_segments_ = number_of_segments;
   }
   /**
            * Total number of segments
            */
   public long getNumberOfSegments()
   {
      return number_of_segments_;
   }

   /**
            * Number of points per segment
            */
   public void setPointsPerSegment(int points_per_segment)
   {
      points_per_segment_ = points_per_segment;
   }
   /**
            * Number of points per segment
            */
   public int getPointsPerSegment()
   {
      return points_per_segment_;
   }

   /**
            * Segment this data represents
            */
   public void setSegmentIndex(long segment_index)
   {
      segment_index_ = segment_index;
   }
   /**
            * Segment this data represents
            */
   public long getSegmentIndex()
   {
      return segment_index_;
   }


   /**
            * Compressed point cloud data
            * Sized specifically for our current primary sensor head streaming
            * 22220 * 4 * 4
            */
   public us.ihmc.idl.IDLSequence.Byte  getScan()
   {
      return scan_;
   }


   public static Supplier<FusedSensorHeadPointCloudMessagePubSubType> getPubSubType()
   {
      return FusedSensorHeadPointCloudMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FusedSensorHeadPointCloudMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FusedSensorHeadPointCloudMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.aquisition_seconds_since_epoch_, other.aquisition_seconds_since_epoch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.aquisition_additional_nanos_, other.aquisition_additional_nanos_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_segments_, other.number_of_segments_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.points_per_segment_, other.points_per_segment_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.segment_index_, other.segment_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.scan_, other.scan_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FusedSensorHeadPointCloudMessage)) return false;

      FusedSensorHeadPointCloudMessage otherMyClass = (FusedSensorHeadPointCloudMessage) other;

      if(this.aquisition_seconds_since_epoch_ != otherMyClass.aquisition_seconds_since_epoch_) return false;

      if(this.aquisition_additional_nanos_ != otherMyClass.aquisition_additional_nanos_) return false;

      if(this.number_of_segments_ != otherMyClass.number_of_segments_) return false;

      if(this.points_per_segment_ != otherMyClass.points_per_segment_) return false;

      if(this.segment_index_ != otherMyClass.segment_index_) return false;

      if (!this.scan_.equals(otherMyClass.scan_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FusedSensorHeadPointCloudMessage {");
      builder.append("aquisition_seconds_since_epoch=");
      builder.append(this.aquisition_seconds_since_epoch_);      builder.append(", ");
      builder.append("aquisition_additional_nanos=");
      builder.append(this.aquisition_additional_nanos_);      builder.append(", ");
      builder.append("number_of_segments=");
      builder.append(this.number_of_segments_);      builder.append(", ");
      builder.append("points_per_segment=");
      builder.append(this.points_per_segment_);      builder.append(", ");
      builder.append("segment_index=");
      builder.append(this.segment_index_);      builder.append(", ");
      builder.append("scan=");
      builder.append(this.scan_);
      builder.append("}");
      return builder.toString();
   }
}
