package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC behavior module.
 */
public class DetectedObjectPacket extends Packet<DetectedObjectPacket> implements Settable<DetectedObjectPacket>, EpsilonComparable<DetectedObjectPacket>
{
   public us.ihmc.euclid.geometry.Pose3D pose_;
   public int id_;

   public DetectedObjectPacket()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public DetectedObjectPacket(DetectedObjectPacket other)
   {
      set(other);
   }

   public void set(DetectedObjectPacket other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      id_ = other.id_;
   }

   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }

   public int getId()
   {
      return id_;
   }

   public void setId(int id)
   {
      id_ = id;
   }

   @Override
   public boolean epsilonEquals(DetectedObjectPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon))
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
      if (!(other instanceof DetectedObjectPacket))
         return false;

      DetectedObjectPacket otherMyClass = (DetectedObjectPacket) other;

      if (!this.pose_.equals(otherMyClass.pose_))
         return false;

      if (this.id_ != otherMyClass.id_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedObjectPacket {");
      builder.append("pose=");
      builder.append(this.pose_);

      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);

      builder.append("}");
      return builder.toString();
   }
}
