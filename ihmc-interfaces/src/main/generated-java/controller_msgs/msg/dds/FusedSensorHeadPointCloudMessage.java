package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FusedSensorHeadPointCloudMessage extends Packet<FusedSensorHeadPointCloudMessage> implements Settable<FusedSensorHeadPointCloudMessage>, EpsilonComparable<FusedSensorHeadPointCloudMessage>
{
   /**
            * Which segment
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
      scan_ = new us.ihmc.idl.IDLSequence.Byte (355520, "type_9");

   }

   public FusedSensorHeadPointCloudMessage(FusedSensorHeadPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(FusedSensorHeadPointCloudMessage other)
   {
      segment_index_ = other.segment_index_;

      scan_.set(other.scan_);
   }

   /**
            * Which segment
            */
   public void setSegmentIndex(long segment_index)
   {
      segment_index_ = segment_index;
   }
   /**
            * Which segment
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

      if(this.segment_index_ != otherMyClass.segment_index_) return false;

      if (!this.scan_.equals(otherMyClass.scan_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FusedSensorHeadPointCloudMessage {");
      builder.append("segment_index=");
      builder.append(this.segment_index_);      builder.append(", ");
      builder.append("scan=");
      builder.append(this.scan_);
      builder.append("}");
      return builder.toString();
   }
}
