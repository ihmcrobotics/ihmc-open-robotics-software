package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC localization module.
 */
public class StampedPosePacket extends Packet<StampedPosePacket> implements Settable<StampedPosePacket>, EpsilonComparable<StampedPosePacket>
{
   public us.ihmc.euclid.geometry.Pose3D pose_;
   public long timestamp_;
   public double confidence_factor_;
   public java.lang.StringBuilder frame_id_;

   public StampedPosePacket()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();

      frame_id_ = new java.lang.StringBuilder(255);
   }

   public StampedPosePacket(StampedPosePacket other)
   {
      set(other);
   }

   public void set(StampedPosePacket other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      timestamp_ = other.timestamp_;

      confidence_factor_ = other.confidence_factor_;

      frame_id_.setLength(0);
      frame_id_.append(other.frame_id_);
   }

   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }

   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }

   public double getConfidenceFactor()
   {
      return confidence_factor_;
   }

   public void setConfidenceFactor(double confidence_factor)
   {
      confidence_factor_ = confidence_factor;
   }

   public java.lang.String getFrameIdAsString()
   {
      return getFrameId().toString();
   }

   public java.lang.StringBuilder getFrameId()
   {
      return frame_id_;
   }

   public void setFrameId(java.lang.String frame_id)
   {
      frame_id_.setLength(0);
      frame_id_.append(frame_id);
   }

   @Override
   public boolean epsilonEquals(StampedPosePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_factor_, other.confidence_factor_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_id_, other.frame_id_, epsilon))
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
      if (!(other instanceof StampedPosePacket))
         return false;

      StampedPosePacket otherMyClass = (StampedPosePacket) other;

      if (!this.pose_.equals(otherMyClass.pose_))
         return false;

      if (this.timestamp_ != otherMyClass.timestamp_)
         return false;

      if (this.confidence_factor_ != otherMyClass.confidence_factor_)
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_id_, otherMyClass.frame_id_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StampedPosePacket {");
      builder.append("pose=");
      builder.append(this.pose_);

      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);

      builder.append(", ");
      builder.append("confidence_factor=");
      builder.append(this.confidence_factor_);

      builder.append(", ");
      builder.append("frame_id=");
      builder.append(this.frame_id_);

      builder.append("}");
      return builder.toString();
   }
}
