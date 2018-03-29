package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket>
      implements Settable<UIPositionCheckerPacket>, EpsilonComparable<UIPositionCheckerPacket>
{
   public us.ihmc.euclid.tuple3D.Point3D position_;
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public UIPositionCheckerPacket()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public UIPositionCheckerPacket(UIPositionCheckerPacket other)
   {
      set(other);
   }

   public void set(UIPositionCheckerPacket other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }

   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }

   @Override
   public boolean epsilonEquals(UIPositionCheckerPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.position_.epsilonEquals(other.position_, epsilon))
         return false;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof UIPositionCheckerPacket))
         return false;

      UIPositionCheckerPacket otherMyClass = (UIPositionCheckerPacket) other;

      if (!this.position_.equals(otherMyClass.position_))
         return false;

      if (!this.orientation_.equals(otherMyClass.orientation_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UIPositionCheckerPacket {");
      builder.append("position=");
      builder.append(this.position_);

      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);

      builder.append("}");
      return builder.toString();
   }
}
