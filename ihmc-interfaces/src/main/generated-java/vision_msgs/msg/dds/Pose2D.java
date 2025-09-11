package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.
       */
public class Pose2D extends Packet<Pose2D> implements Settable<Pose2D>, EpsilonComparable<Pose2D>
{
   public vision_msgs.msg.dds.Point2D position_;
   public double theta_;

   public Pose2D()
   {
      position_ = new vision_msgs.msg.dds.Point2D();
   }

   public Pose2D(Pose2D other)
   {
      this();
      set(other);
   }

   public void set(Pose2D other)
   {
      vision_msgs.msg.dds.Point2DPubSubType.staticCopy(other.position_, position_);
      theta_ = other.theta_;

   }


   public vision_msgs.msg.dds.Point2D getPosition()
   {
      return position_;
   }

   public void setTheta(double theta)
   {
      theta_ = theta;
   }
   public double getTheta()
   {
      return theta_;
   }


   public static Supplier<Pose2DPubSubType> getPubSubType()
   {
      return Pose2DPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Pose2DPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Pose2D other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.theta_, other.theta_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Pose2D)) return false;

      Pose2D otherMyClass = (Pose2D) other;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if(this.theta_ != otherMyClass.theta_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Pose2D {");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("theta=");
      builder.append(this.theta_);
      builder.append("}");
      return builder.toString();
   }
}
