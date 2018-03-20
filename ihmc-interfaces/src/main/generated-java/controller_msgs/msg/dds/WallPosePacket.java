package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class WallPosePacket extends Packet<WallPosePacket> implements Settable<WallPosePacket>, EpsilonComparable<WallPosePacket>
{
   public double cutting_radius_ = 0.2;
   public us.ihmc.euclid.tuple3D.Point3D center_position_;
   public us.ihmc.euclid.tuple4D.Quaternion center_orientation_;

   public WallPosePacket()
   {

      center_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      center_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public WallPosePacket(WallPosePacket other)
   {
      set(other);
   }

   public void set(WallPosePacket other)
   {
      cutting_radius_ = other.cutting_radius_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.center_position_, center_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.center_orientation_, center_orientation_);
   }

   public double getCuttingRadius()
   {
      return cutting_radius_;
   }

   public void setCuttingRadius(double cutting_radius)
   {
      cutting_radius_ = cutting_radius;
   }

   public us.ihmc.euclid.tuple3D.Point3D getCenterPosition()
   {
      return center_position_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getCenterOrientation()
   {
      return center_orientation_;
   }

   @Override
   public boolean epsilonEquals(WallPosePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cutting_radius_, other.cutting_radius_, epsilon))
         return false;

      if (!this.center_position_.epsilonEquals(other.center_position_, epsilon))
         return false;

      if (!this.center_orientation_.epsilonEquals(other.center_orientation_, epsilon))
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
      if (!(other instanceof WallPosePacket))
         return false;

      WallPosePacket otherMyClass = (WallPosePacket) other;

      if (this.cutting_radius_ != otherMyClass.cutting_radius_)
         return false;

      if (!this.center_position_.equals(otherMyClass.center_position_))
         return false;

      if (!this.center_orientation_.equals(otherMyClass.center_orientation_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WallPosePacket {");
      builder.append("cutting_radius=");
      builder.append(this.cutting_radius_);

      builder.append(", ");
      builder.append("center_position=");
      builder.append(this.center_position_);

      builder.append(", ");
      builder.append("center_orientation=");
      builder.append(this.center_orientation_);

      builder.append("}");
      return builder.toString();
   }
}