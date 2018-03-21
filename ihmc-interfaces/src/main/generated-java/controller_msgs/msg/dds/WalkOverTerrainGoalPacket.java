package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class WalkOverTerrainGoalPacket implements Settable<WalkOverTerrainGoalPacket>, EpsilonComparable<WalkOverTerrainGoalPacket>
{
   private us.ihmc.euclid.tuple3D.Point3D position_;
   private us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public WalkOverTerrainGoalPacket()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public WalkOverTerrainGoalPacket(WalkOverTerrainGoalPacket other)
   {
      set(other);
   }

   public void set(WalkOverTerrainGoalPacket other)
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
   public boolean epsilonEquals(WalkOverTerrainGoalPacket other, double epsilon)
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
      if (!(other instanceof WalkOverTerrainGoalPacket))
         return false;

      WalkOverTerrainGoalPacket otherMyClass = (WalkOverTerrainGoalPacket) other;

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

      builder.append("WalkOverTerrainGoalPacket {");
      builder.append("position=");
      builder.append(this.position_);

      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);

      builder.append("}");
      return builder.toString();
   }
}