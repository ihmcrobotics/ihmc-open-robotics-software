package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Image32 extends Packet<Image32> implements Settable<Image32>, EpsilonComparable<Image32>
{
   /**
            * This message can be used as a general image.
            * Each component of the data which of length is same with width * height is an integer value describing color with the conventional RGB.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public int width_;
   public int height_;
   public us.ihmc.idl.IDLSequence.Integer  rgbdata_;

   public Image32()
   {
      rgbdata_ = new us.ihmc.idl.IDLSequence.Integer (4000000, "type_2");

   }

   public Image32(Image32 other)
   {
      this();
      set(other);
   }

   public void set(Image32 other)
   {
      sequence_id_ = other.sequence_id_;

      width_ = other.width_;

      height_ = other.height_;

      rgbdata_.set(other.rgbdata_);
   }

   /**
            * This message can be used as a general image.
            * Each component of the data which of length is same with width * height is an integer value describing color with the conventional RGB.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * This message can be used as a general image.
            * Each component of the data which of length is same with width * height is an integer value describing color with the conventional RGB.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setWidth(int width)
   {
      width_ = width;
   }
   public int getWidth()
   {
      return width_;
   }

   public void setHeight(int height)
   {
      height_ = height;
   }
   public int getHeight()
   {
      return height_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getRgbdata()
   {
      return rgbdata_;
   }


   public static Supplier<Image32PubSubType> getPubSubType()
   {
      return Image32PubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Image32PubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Image32 other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_, other.width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.rgbdata_, other.rgbdata_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Image32)) return false;

      Image32 otherMyClass = (Image32) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.width_ != otherMyClass.width_) return false;

      if(this.height_ != otherMyClass.height_) return false;

      if (!this.rgbdata_.equals(otherMyClass.rgbdata_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Image32 {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("width=");
      builder.append(this.width_);      builder.append(", ");
      builder.append("height=");
      builder.append(this.height_);      builder.append(", ");
      builder.append("rgbdata=");
      builder.append(this.rgbdata_);
      builder.append("}");
      return builder.toString();
   }
}
