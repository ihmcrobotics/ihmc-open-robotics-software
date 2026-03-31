package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Point2DArray extends Packet<Point2DArray> implements Settable<Point2DArray>, EpsilonComparable<Point2DArray>
{
   /**
            * Array of 2D points
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.Point2D>  points_;

   public Point2DArray()
   {
      points_ = new us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.Point2D> (100, new vision_msgs.msg.dds.Point2DPubSubType());

   }

   public Point2DArray(Point2DArray other)
   {
      this();
      set(other);
   }

   public void set(Point2DArray other)
   {
      points_.set(other.points_);
   }


   /**
            * Array of 2D points
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.Point2D>  getPoints()
   {
      return points_;
   }


   public static Supplier<Point2DArrayPubSubType> getPubSubType()
   {
      return Point2DArrayPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Point2DArrayPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Point2DArray other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.points_.size() != other.points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.points_.size(); i++)
         {  if (!this.points_.get(i).epsilonEquals(other.points_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Point2DArray)) return false;

      Point2DArray otherMyClass = (Point2DArray) other;

      if (!this.points_.equals(otherMyClass.points_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Point2DArray {");
      builder.append("points=");
      builder.append(this.points_);
      builder.append("}");
      return builder.toString();
   }
}
