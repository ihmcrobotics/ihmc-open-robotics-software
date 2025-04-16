package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PathPoint extends Packet<PathPoint> implements Settable<PathPoint>, EpsilonComparable<PathPoint>
{
   public float t_from_start_;
   public float x_;
   public float y_;
   public float yaw_;
   public float vx_;
   public float vy_;
   public float vyaw_;

   public PathPoint()
   {
   }

   public PathPoint(PathPoint other)
   {
      this();
      set(other);
   }

   public void set(PathPoint other)
   {
      t_from_start_ = other.t_from_start_;

      x_ = other.x_;

      y_ = other.y_;

      yaw_ = other.yaw_;

      vx_ = other.vx_;

      vy_ = other.vy_;

      vyaw_ = other.vyaw_;

   }

   public void setTFromStart(float t_from_start)
   {
      t_from_start_ = t_from_start;
   }
   public float getTFromStart()
   {
      return t_from_start_;
   }

   public void setX(float x)
   {
      x_ = x;
   }
   public float getX()
   {
      return x_;
   }

   public void setY(float y)
   {
      y_ = y;
   }
   public float getY()
   {
      return y_;
   }

   public void setYaw(float yaw)
   {
      yaw_ = yaw;
   }
   public float getYaw()
   {
      return yaw_;
   }

   public void setVx(float vx)
   {
      vx_ = vx;
   }
   public float getVx()
   {
      return vx_;
   }

   public void setVy(float vy)
   {
      vy_ = vy;
   }
   public float getVy()
   {
      return vy_;
   }

   public void setVyaw(float vyaw)
   {
      vyaw_ = vyaw;
   }
   public float getVyaw()
   {
      return vyaw_;
   }


   public static Supplier<PathPointPubSubType> getPubSubType()
   {
      return PathPointPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PathPointPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PathPoint other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.t_from_start_, other.t_from_start_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_, other.x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_, other.y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_, other.yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vx_, other.vx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vy_, other.vy_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vyaw_, other.vyaw_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PathPoint)) return false;

      PathPoint otherMyClass = (PathPoint) other;

      if(this.t_from_start_ != otherMyClass.t_from_start_) return false;

      if(this.x_ != otherMyClass.x_) return false;

      if(this.y_ != otherMyClass.y_) return false;

      if(this.yaw_ != otherMyClass.yaw_) return false;

      if(this.vx_ != otherMyClass.vx_) return false;

      if(this.vy_ != otherMyClass.vy_) return false;

      if(this.vyaw_ != otherMyClass.vyaw_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PathPoint {");
      builder.append("t_from_start=");
      builder.append(this.t_from_start_);      builder.append(", ");
      builder.append("x=");
      builder.append(this.x_);      builder.append(", ");
      builder.append("y=");
      builder.append(this.y_);      builder.append(", ");
      builder.append("yaw=");
      builder.append(this.yaw_);      builder.append(", ");
      builder.append("vx=");
      builder.append(this.vx_);      builder.append(", ");
      builder.append("vy=");
      builder.append(this.vy_);      builder.append(", ");
      builder.append("vyaw=");
      builder.append(this.vyaw_);
      builder.append("}");
      return builder.toString();
   }
}
