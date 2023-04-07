package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SteppableRegionDebugImageMessage extends Packet<SteppableRegionDebugImageMessage> implements Settable<SteppableRegionDebugImageMessage>, EpsilonComparable<SteppableRegionDebugImageMessage>
{
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

   public SteppableRegionDebugImageMessage()
   {
      data_ = new us.ihmc.idl.IDLSequence.Byte (250000, "type_9");

   }

   public SteppableRegionDebugImageMessage(SteppableRegionDebugImageMessage other)
   {
      this();
      set(other);
   }

   public void set(SteppableRegionDebugImageMessage other)
   {
      image_width_ = other.image_width_;

      image_height_ = other.image_height_;

      data_.set(other.data_);
      format_ = other.format_;

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


   public static Supplier<SteppableRegionDebugImageMessagePubSubType> getPubSubType()
   {
      return SteppableRegionDebugImageMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SteppableRegionDebugImageMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SteppableRegionDebugImageMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_width_, other.image_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_height_, other.image_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.format_, other.format_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SteppableRegionDebugImageMessage)) return false;

      SteppableRegionDebugImageMessage otherMyClass = (SteppableRegionDebugImageMessage) other;

      if(this.image_width_ != otherMyClass.image_width_) return false;

      if(this.image_height_ != otherMyClass.image_height_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;
      if(this.format_ != otherMyClass.format_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SteppableRegionDebugImageMessage {");
      builder.append("image_width=");
      builder.append(this.image_width_);      builder.append(", ");
      builder.append("image_height=");
      builder.append(this.image_height_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");
      builder.append("format=");
      builder.append(this.format_);
      builder.append("}");
      return builder.toString();
   }
}
