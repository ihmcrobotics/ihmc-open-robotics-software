package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HeightMap extends Packet<HeightMap> implements Settable<HeightMap>, EpsilonComparable<HeightMap>
{
   /**
            * Header
            * timestamp
            */
   public double stamp_;
   /**
            * world frame id
            */
   public java.lang.StringBuilder frame_id_;
   /**
            * Map info
            * The map resolution [m/cell]
            */
   public float resolution_;
   /**
            * Map width along x-axis [cells]
            */
   public long width_;
   /**
            * Map height alonge y-axis [cells]
            */
   public long height_;
   /**
            * Map frame origin xy-position [m], the xyz-axis direction of map frame is aligned with the world frame
            */
   public float[] origin_;
   /**
            * Map data, in x-major order, starting with [0,0], ending with [width, height]
            * For a cell whose 2d-array-index is [ix, iy]，
            * its position in world frame is: [ix * resolution + origin[0], iy * resolution + origin[1]]
            * its cell value is: data[width * iy + ix]
            */
   public us.ihmc.idl.IDLSequence.Float  data_;

   public HeightMap()
   {
      frame_id_ = new java.lang.StringBuilder(255);
      origin_ = new float[2];

      data_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

   }

   public HeightMap(HeightMap other)
   {
      this();
      set(other);
   }

   public void set(HeightMap other)
   {
      stamp_ = other.stamp_;

      frame_id_.setLength(0);
      frame_id_.append(other.frame_id_);

      resolution_ = other.resolution_;

      width_ = other.width_;

      height_ = other.height_;

      for(int i1 = 0; i1 < origin_.length; ++i1)
      {
            origin_[i1] = other.origin_[i1];

      }

      data_.set(other.data_);
   }

   /**
            * Header
            * timestamp
            */
   public void setStamp(double stamp)
   {
      stamp_ = stamp;
   }
   /**
            * Header
            * timestamp
            */
   public double getStamp()
   {
      return stamp_;
   }

   /**
            * world frame id
            */
   public void setFrameId(java.lang.String frame_id)
   {
      frame_id_.setLength(0);
      frame_id_.append(frame_id);
   }

   /**
            * world frame id
            */
   public java.lang.String getFrameIdAsString()
   {
      return getFrameId().toString();
   }
   /**
            * world frame id
            */
   public java.lang.StringBuilder getFrameId()
   {
      return frame_id_;
   }

   /**
            * Map info
            * The map resolution [m/cell]
            */
   public void setResolution(float resolution)
   {
      resolution_ = resolution;
   }
   /**
            * Map info
            * The map resolution [m/cell]
            */
   public float getResolution()
   {
      return resolution_;
   }

   /**
            * Map width along x-axis [cells]
            */
   public void setWidth(long width)
   {
      width_ = width;
   }
   /**
            * Map width along x-axis [cells]
            */
   public long getWidth()
   {
      return width_;
   }

   /**
            * Map height alonge y-axis [cells]
            */
   public void setHeight(long height)
   {
      height_ = height;
   }
   /**
            * Map height alonge y-axis [cells]
            */
   public long getHeight()
   {
      return height_;
   }


   /**
            * Map frame origin xy-position [m], the xyz-axis direction of map frame is aligned with the world frame
            */
   public float[] getOrigin()
   {
      return origin_;
   }


   /**
            * Map data, in x-major order, starting with [0,0], ending with [width, height]
            * For a cell whose 2d-array-index is [ix, iy]，
            * its position in world frame is: [ix * resolution + origin[0], iy * resolution + origin[1]]
            * its cell value is: data[width * iy + ix]
            */
   public us.ihmc.idl.IDLSequence.Float  getData()
   {
      return data_;
   }


   public static Supplier<HeightMapPubSubType> getPubSubType()
   {
      return HeightMapPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeightMapPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeightMap other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stamp_, other.stamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_id_, other.frame_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resolution_, other.resolution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_, other.width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon)) return false;

      for(int i3 = 0; i3 < origin_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.origin_[i3], other.origin_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.data_, other.data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeightMap)) return false;

      HeightMap otherMyClass = (HeightMap) other;

      if(this.stamp_ != otherMyClass.stamp_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_id_, otherMyClass.frame_id_)) return false;

      if(this.resolution_ != otherMyClass.resolution_) return false;

      if(this.width_ != otherMyClass.width_) return false;

      if(this.height_ != otherMyClass.height_) return false;

      for(int i5 = 0; i5 < origin_.length; ++i5)
      {
                if(this.origin_[i5] != otherMyClass.origin_[i5]) return false;

      }
      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightMap {");
      builder.append("stamp=");
      builder.append(this.stamp_);      builder.append(", ");
      builder.append("frame_id=");
      builder.append(this.frame_id_);      builder.append(", ");
      builder.append("resolution=");
      builder.append(this.resolution_);      builder.append(", ");
      builder.append("width=");
      builder.append(this.width_);      builder.append(", ");
      builder.append("height=");
      builder.append(this.height_);      builder.append(", ");
      builder.append("origin=");
      builder.append(java.util.Arrays.toString(this.origin_));      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
