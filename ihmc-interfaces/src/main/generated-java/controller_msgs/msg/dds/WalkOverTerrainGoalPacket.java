package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class WalkOverTerrainGoalPacket extends Packet<WalkOverTerrainGoalPacket>
      implements Settable<WalkOverTerrainGoalPacket>, EpsilonComparable<WalkOverTerrainGoalPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.euclid.tuple3D.Point3D position_;
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public WalkOverTerrainGoalPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public WalkOverTerrainGoalPacket(WalkOverTerrainGoalPacket other)
   {
      this();
      set(other);
   }

   public void set(WalkOverTerrainGoalPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
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

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
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
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);
      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
