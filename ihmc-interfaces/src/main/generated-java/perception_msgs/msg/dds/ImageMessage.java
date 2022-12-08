package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Image message. It has a large capacity.
       * It's recommended to reuse instances to avoid excessive memory allocation.
       */
public class ImageMessage extends Packet<ImageMessage> implements Settable<ImageMessage>, EpsilonComparable<ImageMessage>
{
   /**
            * Sequence number. Used for detecting out of order or dropped messages
            */
   public long sequence_number_;
   /**
            * Use precise time measurements to measure delay between processes
            */
   public long acquisition_time_seconds_since_epoch_;
   /**
            * Use precise time measurements to measure delay between processes
            */
   public long acquisition_time_additional_nanos_;
   /**
            * Image width in pixels
            */
   public int image_width_;
   /**
            * Image height in pixels
            */
   public int image_height_;
   /**
            * The raw data for the image
            */
   public us.ihmc.idl.IDLSequence.Byte  data_;
   /**
            * The image format. Ordinal of OpenCVImageFormat
            */
   public int format_;
   /**
            * Position of the focal point at sensor data aquisition
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Orientation of the focal point at sensor data aquisition
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public ImageMessage()
   {
      data_ = new us.ihmc.idl.IDLSequence.Byte (25000000, "type_9");

      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public ImageMessage(ImageMessage other)
   {
      this();
      set(other);
   }

   public void set(ImageMessage other)
   {
      sequence_number_ = other.sequence_number_;

      acquisition_time_seconds_since_epoch_ = other.acquisition_time_seconds_since_epoch_;

      acquisition_time_additional_nanos_ = other.acquisition_time_additional_nanos_;

      image_width_ = other.image_width_;

      image_height_ = other.image_height_;

      data_.set(other.data_);
      format_ = other.format_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }

   /**
            * Sequence number. Used for detecting out of order or dropped messages
            */
   public void setSequenceNumber(long sequence_number)
   {
      sequence_number_ = sequence_number;
   }
   /**
            * Sequence number. Used for detecting out of order or dropped messages
            */
   public long getSequenceNumber()
   {
      return sequence_number_;
   }

   /**
            * Use precise time measurements to measure delay between processes
            */
   public void setAcquisitionTimeSecondsSinceEpoch(long acquisition_time_seconds_since_epoch)
   {
      acquisition_time_seconds_since_epoch_ = acquisition_time_seconds_since_epoch;
   }
   /**
            * Use precise time measurements to measure delay between processes
            */
   public long getAcquisitionTimeSecondsSinceEpoch()
   {
      return acquisition_time_seconds_since_epoch_;
   }

   /**
            * Use precise time measurements to measure delay between processes
            */
   public void setAcquisitionTimeAdditionalNanos(long acquisition_time_additional_nanos)
   {
      acquisition_time_additional_nanos_ = acquisition_time_additional_nanos;
   }
   /**
            * Use precise time measurements to measure delay between processes
            */
   public long getAcquisitionTimeAdditionalNanos()
   {
      return acquisition_time_additional_nanos_;
   }

   /**
            * Image width in pixels
            */
   public void setImageWidth(int image_width)
   {
      image_width_ = image_width;
   }
   /**
            * Image width in pixels
            */
   public int getImageWidth()
   {
      return image_width_;
   }

   /**
            * Image height in pixels
            */
   public void setImageHeight(int image_height)
   {
      image_height_ = image_height;
   }
   /**
            * Image height in pixels
            */
   public int getImageHeight()
   {
      return image_height_;
   }


   /**
            * The raw data for the image
            */
   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }

   /**
            * The image format. Ordinal of OpenCVImageFormat
            */
   public void setFormat(int format)
   {
      format_ = format;
   }
   /**
            * The image format. Ordinal of OpenCVImageFormat
            */
   public int getFormat()
   {
      return format_;
   }


   /**
            * Position of the focal point at sensor data aquisition
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   /**
            * Orientation of the focal point at sensor data aquisition
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   public static Supplier<ImageMessagePubSubType> getPubSubType()
   {
      return ImageMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ImageMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ImageMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_number_, other.sequence_number_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acquisition_time_seconds_since_epoch_, other.acquisition_time_seconds_since_epoch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acquisition_time_additional_nanos_, other.acquisition_time_additional_nanos_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_width_, other.image_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_height_, other.image_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.format_, other.format_, epsilon)) return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ImageMessage)) return false;

      ImageMessage otherMyClass = (ImageMessage) other;

      if(this.sequence_number_ != otherMyClass.sequence_number_) return false;

      if(this.acquisition_time_seconds_since_epoch_ != otherMyClass.acquisition_time_seconds_since_epoch_) return false;

      if(this.acquisition_time_additional_nanos_ != otherMyClass.acquisition_time_additional_nanos_) return false;

      if(this.image_width_ != otherMyClass.image_width_) return false;

      if(this.image_height_ != otherMyClass.image_height_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;
      if(this.format_ != otherMyClass.format_) return false;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ImageMessage {");
      builder.append("sequence_number=");
      builder.append(this.sequence_number_);      builder.append(", ");
      builder.append("acquisition_time_seconds_since_epoch=");
      builder.append(this.acquisition_time_seconds_since_epoch_);      builder.append(", ");
      builder.append("acquisition_time_additional_nanos=");
      builder.append(this.acquisition_time_additional_nanos_);      builder.append(", ");
      builder.append("image_width=");
      builder.append(this.image_width_);      builder.append(", ");
      builder.append("image_height=");
      builder.append(this.image_height_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");
      builder.append("format=");
      builder.append(this.format_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
