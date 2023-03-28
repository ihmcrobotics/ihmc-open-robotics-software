package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Big video packet. Supports up to RGB 8-bit 4K uncompressed images.
       */
public class BigVideoPacket extends Packet<BigVideoPacket> implements Settable<BigVideoPacket>, EpsilonComparable<BigVideoPacket>
{
   public long acquisition_time_seconds_since_epoch_;
   public long acquisition_time_additional_nanos_;
   public int image_width_;
   public int image_height_;
   public us.ihmc.idl.IDLSequence.Byte  data_;

   public BigVideoPacket()
   {
      data_ = new us.ihmc.idl.IDLSequence.Byte (25000000, "type_9");

   }

   public BigVideoPacket(BigVideoPacket other)
   {
      this();
      set(other);
   }

   public void set(BigVideoPacket other)
   {
      acquisition_time_seconds_since_epoch_ = other.acquisition_time_seconds_since_epoch_;

      acquisition_time_additional_nanos_ = other.acquisition_time_additional_nanos_;

      image_width_ = other.image_width_;

      image_height_ = other.image_height_;

      data_.set(other.data_);
   }

   public void setAcquisitionTimeSecondsSinceEpoch(long acquisition_time_seconds_since_epoch)
   {
      acquisition_time_seconds_since_epoch_ = acquisition_time_seconds_since_epoch;
   }
   public long getAcquisitionTimeSecondsSinceEpoch()
   {
      return acquisition_time_seconds_since_epoch_;
   }

   public void setAcquisitionTimeAdditionalNanos(long acquisition_time_additional_nanos)
   {
      acquisition_time_additional_nanos_ = acquisition_time_additional_nanos;
   }
   public long getAcquisitionTimeAdditionalNanos()
   {
      return acquisition_time_additional_nanos_;
   }

   public void setImageWidth(int image_width)
   {
      image_width_ = image_width;
   }
   public int getImageWidth()
   {
      return image_width_;
   }

   public void setImageHeight(int image_height)
   {
      image_height_ = image_height;
   }
   public int getImageHeight()
   {
      return image_height_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }


   public static Supplier<BigVideoPacketPubSubType> getPubSubType()
   {
      return BigVideoPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BigVideoPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BigVideoPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acquisition_time_seconds_since_epoch_, other.acquisition_time_seconds_since_epoch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acquisition_time_additional_nanos_, other.acquisition_time_additional_nanos_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_width_, other.image_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_height_, other.image_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BigVideoPacket)) return false;

      BigVideoPacket otherMyClass = (BigVideoPacket) other;

      if(this.acquisition_time_seconds_since_epoch_ != otherMyClass.acquisition_time_seconds_since_epoch_) return false;

      if(this.acquisition_time_additional_nanos_ != otherMyClass.acquisition_time_additional_nanos_) return false;

      if(this.image_width_ != otherMyClass.image_width_) return false;

      if(this.image_height_ != otherMyClass.image_height_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BigVideoPacket {");
      builder.append("acquisition_time_seconds_since_epoch=");
      builder.append(this.acquisition_time_seconds_since_epoch_);      builder.append(", ");
      builder.append("acquisition_time_additional_nanos=");
      builder.append(this.acquisition_time_additional_nanos_);      builder.append(", ");
      builder.append("image_width=");
      builder.append(this.image_width_);      builder.append(", ");
      builder.append("image_height=");
      builder.append(this.image_height_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
