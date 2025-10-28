package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A 2D bounding box that can be rotated about its center.
       * All dimensions are in pixels, but represented using floating-point
       * values to allow sub-pixel precision. If an exact pixel crop is required
       * for a rotated bounding box, it can be calculated using Bresenham's line
       * algorithm.
       */
public class BoundingBox2D extends Packet<BoundingBox2D> implements Settable<BoundingBox2D>, EpsilonComparable<BoundingBox2D>
{
   /**
            * The 2D position (in pixels) and orientation of the bounding box center.
            */
   public vision_msgs.msg.dds.Pose2D center_;
   /**
            * The total size (in pixels) of the bounding box surrounding the object relative
            * to the pose of its center.
            */
   public double size_x_;
   public double size_y_;

   public BoundingBox2D()
   {
      center_ = new vision_msgs.msg.dds.Pose2D();
   }

   public BoundingBox2D(BoundingBox2D other)
   {
      this();
      set(other);
   }

   public void set(BoundingBox2D other)
   {
      vision_msgs.msg.dds.Pose2DPubSubType.staticCopy(other.center_, center_);
      size_x_ = other.size_x_;

      size_y_ = other.size_y_;

   }


   /**
            * The 2D position (in pixels) and orientation of the bounding box center.
            */
   public vision_msgs.msg.dds.Pose2D getCenter()
   {
      return center_;
   }

   /**
            * The total size (in pixels) of the bounding box surrounding the object relative
            * to the pose of its center.
            */
   public void setSizeX(double size_x)
   {
      size_x_ = size_x;
   }
   /**
            * The total size (in pixels) of the bounding box surrounding the object relative
            * to the pose of its center.
            */
   public double getSizeX()
   {
      return size_x_;
   }

   public void setSizeY(double size_y)
   {
      size_y_ = size_y;
   }
   public double getSizeY()
   {
      return size_y_;
   }


   public static Supplier<BoundingBox2DPubSubType> getPubSubType()
   {
      return BoundingBox2DPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BoundingBox2DPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BoundingBox2D other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.center_.epsilonEquals(other.center_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.size_x_, other.size_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.size_y_, other.size_y_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BoundingBox2D)) return false;

      BoundingBox2D otherMyClass = (BoundingBox2D) other;

      if (!this.center_.equals(otherMyClass.center_)) return false;
      if(this.size_x_ != otherMyClass.size_x_) return false;

      if(this.size_y_ != otherMyClass.size_y_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BoundingBox2D {");
      builder.append("center=");
      builder.append(this.center_);      builder.append(", ");
      builder.append("size_x=");
      builder.append(this.size_x_);      builder.append(", ");
      builder.append("size_y=");
      builder.append(this.size_y_);
      builder.append("}");
      return builder.toString();
   }
}
