package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Represents a 2D point in pixel coordinates.
       * XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.
       */
public class Point2D extends Packet<Point2D> implements Settable<Point2D>, EpsilonComparable<Point2D>
{
   public double x_;
   public double y_;

   public Point2D()
   {
   }

   public Point2D(Point2D other)
   {
      this();
      set(other);
   }

   public void set(Point2D other)
   {
      x_ = other.x_;

      y_ = other.y_;

   }

   public void setX(double x)
   {
      x_ = x;
   }
   public double getX()
   {
      return x_;
   }

   public void setY(double y)
   {
      y_ = y;
   }
   public double getY()
   {
      return y_;
   }


   public static Supplier<Point2DPubSubType> getPubSubType()
   {
      return Point2DPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Point2DPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Point2D other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_, other.x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_, other.y_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Point2D)) return false;

      Point2D otherMyClass = (Point2D) other;

      if(this.x_ != otherMyClass.x_) return false;

      if(this.y_ != otherMyClass.y_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Point2D {");
      builder.append("x=");
      builder.append(this.x_);      builder.append(", ");
      builder.append("y=");
      builder.append(this.y_);
      builder.append("}");
      return builder.toString();
   }
}
