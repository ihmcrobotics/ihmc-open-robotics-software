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
            * The instant right after we got data from the sensor. Used to measure delay between processes
            */
   public ihmc_common_msgs.msg.dds.InstantMessage acquisition_time_;
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
   /**
            * Is of the pinhole camera model (See https://en.wikipedia.org/wiki/Pinhole_camera_model
            */
   public boolean is_pinhole_camera_model_;
   /**
            * Is equidistant fisheye camera model (See https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function)
            */
   public boolean is_equidistant_fisheye_camera_model_;
   /**
            * Horizontal focal length in units of pixels (Fx)
            */
   public float focal_length_x_pixels_;
   /**
            * Vertical focal length in units of pixels (Fx)
            */
   public float focal_length_y_pixels_;
   /**
            * Principal point X in units of pixels (Cx)
            */
   public float principal_point_x_pixels_;
   /**
            * Principal point Y in units of pixels (Cy)
            */
   public float principal_point_y_pixels_;

   public ImageMessage()
   {
      acquisition_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
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

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.acquisition_time_, acquisition_time_);
      image_width_ = other.image_width_;

      image_height_ = other.image_height_;

      data_.set(other.data_);
      format_ = other.format_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      is_pinhole_camera_model_ = other.is_pinhole_camera_model_;

      is_equidistant_fisheye_camera_model_ = other.is_equidistant_fisheye_camera_model_;

      focal_length_x_pixels_ = other.focal_length_x_pixels_;

      focal_length_y_pixels_ = other.focal_length_y_pixels_;

      principal_point_x_pixels_ = other.principal_point_x_pixels_;

      principal_point_y_pixels_ = other.principal_point_y_pixels_;

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
            * The instant right after we got data from the sensor. Used to measure delay between processes
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getAcquisitionTime()
   {
      return acquisition_time_;
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

   /**
            * Is of the pinhole camera model (See https://en.wikipedia.org/wiki/Pinhole_camera_model
            */
   public void setIsPinholeCameraModel(boolean is_pinhole_camera_model)
   {
      is_pinhole_camera_model_ = is_pinhole_camera_model;
   }
   /**
            * Is of the pinhole camera model (See https://en.wikipedia.org/wiki/Pinhole_camera_model
            */
   public boolean getIsPinholeCameraModel()
   {
      return is_pinhole_camera_model_;
   }

   /**
            * Is equidistant fisheye camera model (See https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function)
            */
   public void setIsEquidistantFisheyeCameraModel(boolean is_equidistant_fisheye_camera_model)
   {
      is_equidistant_fisheye_camera_model_ = is_equidistant_fisheye_camera_model;
   }
   /**
            * Is equidistant fisheye camera model (See https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function)
            */
   public boolean getIsEquidistantFisheyeCameraModel()
   {
      return is_equidistant_fisheye_camera_model_;
   }

   /**
            * Horizontal focal length in units of pixels (Fx)
            */
   public void setFocalLengthXPixels(float focal_length_x_pixels)
   {
      focal_length_x_pixels_ = focal_length_x_pixels;
   }
   /**
            * Horizontal focal length in units of pixels (Fx)
            */
   public float getFocalLengthXPixels()
   {
      return focal_length_x_pixels_;
   }

   /**
            * Vertical focal length in units of pixels (Fx)
            */
   public void setFocalLengthYPixels(float focal_length_y_pixels)
   {
      focal_length_y_pixels_ = focal_length_y_pixels;
   }
   /**
            * Vertical focal length in units of pixels (Fx)
            */
   public float getFocalLengthYPixels()
   {
      return focal_length_y_pixels_;
   }

   /**
            * Principal point X in units of pixels (Cx)
            */
   public void setPrincipalPointXPixels(float principal_point_x_pixels)
   {
      principal_point_x_pixels_ = principal_point_x_pixels;
   }
   /**
            * Principal point X in units of pixels (Cx)
            */
   public float getPrincipalPointXPixels()
   {
      return principal_point_x_pixels_;
   }

   /**
            * Principal point Y in units of pixels (Cy)
            */
   public void setPrincipalPointYPixels(float principal_point_y_pixels)
   {
      principal_point_y_pixels_ = principal_point_y_pixels;
   }
   /**
            * Principal point Y in units of pixels (Cy)
            */
   public float getPrincipalPointYPixels()
   {
      return principal_point_y_pixels_;
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

      if (!this.acquisition_time_.epsilonEquals(other.acquisition_time_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_width_, other.image_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_height_, other.image_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.format_, other.format_, epsilon)) return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_pinhole_camera_model_, other.is_pinhole_camera_model_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_equidistant_fisheye_camera_model_, other.is_equidistant_fisheye_camera_model_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.focal_length_x_pixels_, other.focal_length_x_pixels_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.focal_length_y_pixels_, other.focal_length_y_pixels_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.principal_point_x_pixels_, other.principal_point_x_pixels_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.principal_point_y_pixels_, other.principal_point_y_pixels_, epsilon)) return false;


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

      if (!this.acquisition_time_.equals(otherMyClass.acquisition_time_)) return false;
      if(this.image_width_ != otherMyClass.image_width_) return false;

      if(this.image_height_ != otherMyClass.image_height_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;
      if(this.format_ != otherMyClass.format_) return false;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if(this.is_pinhole_camera_model_ != otherMyClass.is_pinhole_camera_model_) return false;

      if(this.is_equidistant_fisheye_camera_model_ != otherMyClass.is_equidistant_fisheye_camera_model_) return false;

      if(this.focal_length_x_pixels_ != otherMyClass.focal_length_x_pixels_) return false;

      if(this.focal_length_y_pixels_ != otherMyClass.focal_length_y_pixels_) return false;

      if(this.principal_point_x_pixels_ != otherMyClass.principal_point_x_pixels_) return false;

      if(this.principal_point_y_pixels_ != otherMyClass.principal_point_y_pixels_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ImageMessage {");
      builder.append("sequence_number=");
      builder.append(this.sequence_number_);      builder.append(", ");
      builder.append("acquisition_time=");
      builder.append(this.acquisition_time_);      builder.append(", ");
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
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("is_pinhole_camera_model=");
      builder.append(this.is_pinhole_camera_model_);      builder.append(", ");
      builder.append("is_equidistant_fisheye_camera_model=");
      builder.append(this.is_equidistant_fisheye_camera_model_);      builder.append(", ");
      builder.append("focal_length_x_pixels=");
      builder.append(this.focal_length_x_pixels_);      builder.append(", ");
      builder.append("focal_length_y_pixels=");
      builder.append(this.focal_length_y_pixels_);      builder.append(", ");
      builder.append("principal_point_x_pixels=");
      builder.append(this.principal_point_x_pixels_);      builder.append(", ");
      builder.append("principal_point_y_pixels=");
      builder.append(this.principal_point_y_pixels_);
      builder.append("}");
      return builder.toString();
   }
}
