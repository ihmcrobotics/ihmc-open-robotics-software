package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Point2DMessage extends Packet<Point2DMessage> implements Settable<Point2DMessage>, EpsilonComparable<Point2DMessage>
{
   /**
            * x coordinate
            */
   public double x_;
   /**
            * y coordinate
            */
   public double y_;

   public Point2DMessage()
   {
   }

   public Point2DMessage(Point2DMessage other)
   {
      this();
      set(other);
   }

   public void set(Point2DMessage other)
   {
      x_ = other.x_;

      y_ = other.y_;

   }

   /**
            * x coordinate
            */
   public void setX(double x)
   {
      x_ = x;
   }
   /**
            * x coordinate
            */
   public double getX()
   {
      return x_;
   }

   /**
            * y coordinate
            */
   public void setY(double y)
   {
      y_ = y;
   }
   /**
            * y coordinate
            */
   public double getY()
   {
      return y_;
   }


   public static Supplier<Point2DMessagePubSubType> getPubSubType()
   {
      return Point2DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Point2DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Point2DMessage other, double epsilon)
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
      if(!(other instanceof Point2DMessage)) return false;

      Point2DMessage otherMyClass = (Point2DMessage) other;

      if(this.x_ != otherMyClass.x_) return false;

      if(this.y_ != otherMyClass.y_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Point2DMessage {");
      builder.append("x=");
      builder.append(this.x_);      builder.append(", ");
      builder.append("y=");
      builder.append(this.y_);
      builder.append("}");
      return builder.toString();
   }
}
