package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * THIS MESSAGE IS SENT OVER VIDEO STREAMS WITH SRT.
       * MAX SIZE IS 881 BYTES. DO NO EXCEED THAT SIZE
       */
public class StreamData extends Packet<StreamData> implements Settable<StreamData>, EpsilonComparable<StreamData>
{
   /**
            * Camera intrinsics
            */
   public int image_width_;
   public int image_height_;
   public float fx_;
   public float fy_;
   public float cx_;
   public float cy_;
   public float depth_discretization_;
   /**
            * Frame acquisition time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage acquisition_time_;
   /**
            * Sensor pose
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public StreamData()
   {
      acquisition_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public StreamData(StreamData other)
   {
      this();
      set(other);
   }

   public void set(StreamData other)
   {
      image_width_ = other.image_width_;

      image_height_ = other.image_height_;

      fx_ = other.fx_;

      fy_ = other.fy_;

      cx_ = other.cx_;

      cy_ = other.cy_;

      depth_discretization_ = other.depth_discretization_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.acquisition_time_, acquisition_time_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }

   /**
            * Camera intrinsics
            */
   public void setImageWidth(int image_width)
   {
      image_width_ = image_width;
   }
   /**
            * Camera intrinsics
            */
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

   public void setFx(float fx)
   {
      fx_ = fx;
   }
   public float getFx()
   {
      return fx_;
   }

   public void setFy(float fy)
   {
      fy_ = fy;
   }
   public float getFy()
   {
      return fy_;
   }

   public void setCx(float cx)
   {
      cx_ = cx;
   }
   public float getCx()
   {
      return cx_;
   }

   public void setCy(float cy)
   {
      cy_ = cy;
   }
   public float getCy()
   {
      return cy_;
   }

   public void setDepthDiscretization(float depth_discretization)
   {
      depth_discretization_ = depth_discretization;
   }
   public float getDepthDiscretization()
   {
      return depth_discretization_;
   }


   /**
            * Frame acquisition time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getAcquisitionTime()
   {
      return acquisition_time_;
   }


   /**
            * Sensor pose
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   public static Supplier<StreamDataPubSubType> getPubSubType()
   {
      return StreamDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StreamDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StreamData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_width_, other.image_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_height_, other.image_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fx_, other.fx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fy_, other.fy_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cx_, other.cx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cy_, other.cy_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.depth_discretization_, other.depth_discretization_, epsilon)) return false;

      if (!this.acquisition_time_.epsilonEquals(other.acquisition_time_, epsilon)) return false;
      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StreamData)) return false;

      StreamData otherMyClass = (StreamData) other;

      if(this.image_width_ != otherMyClass.image_width_) return false;

      if(this.image_height_ != otherMyClass.image_height_) return false;

      if(this.fx_ != otherMyClass.fx_) return false;

      if(this.fy_ != otherMyClass.fy_) return false;

      if(this.cx_ != otherMyClass.cx_) return false;

      if(this.cy_ != otherMyClass.cy_) return false;

      if(this.depth_discretization_ != otherMyClass.depth_discretization_) return false;

      if (!this.acquisition_time_.equals(otherMyClass.acquisition_time_)) return false;
      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StreamData {");
      builder.append("image_width=");
      builder.append(this.image_width_);      builder.append(", ");
      builder.append("image_height=");
      builder.append(this.image_height_);      builder.append(", ");
      builder.append("fx=");
      builder.append(this.fx_);      builder.append(", ");
      builder.append("fy=");
      builder.append(this.fy_);      builder.append(", ");
      builder.append("cx=");
      builder.append(this.cx_);      builder.append(", ");
      builder.append("cy=");
      builder.append(this.cy_);      builder.append(", ");
      builder.append("depth_discretization=");
      builder.append(this.depth_discretization_);      builder.append(", ");
      builder.append("acquisition_time=");
      builder.append(this.acquisition_time_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
