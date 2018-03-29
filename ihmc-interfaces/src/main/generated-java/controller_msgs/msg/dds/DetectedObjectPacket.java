package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC behavior module.
 */
public class DetectedObjectPacket extends Packet<DetectedObjectPacket> implements Settable<DetectedObjectPacket>, EpsilonComparable<DetectedObjectPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.euclid.geometry.Pose3D pose_;
   public int id_;

   public DetectedObjectPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public DetectedObjectPacket(DetectedObjectPacket other)
   {
      this();
      set(other);
   }

   public void set(DetectedObjectPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      id_ = other.id_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }

   public void setId(int id)
   {
      id_ = id;
   }

   public int getId()
   {
      return id_;
   }

   @Override
   public boolean epsilonEquals(DetectedObjectPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
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
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);
      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);
      builder.append("}");
      return builder.toString();
   }
}
