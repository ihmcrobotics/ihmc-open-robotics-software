package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is obsolete
 */
public class PointCloudWorldPacket extends Packet<PointCloudWorldPacket> implements Settable<PointCloudWorldPacket>, EpsilonComparable<PointCloudWorldPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public long timestamp_;
   public us.ihmc.idl.IDLSequence.Float ground_quad_tree_support_;
   public us.ihmc.idl.IDLSequence.Float decaying_world_scan_;
   public float default_ground_height_;

   public PointCloudWorldPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      ground_quad_tree_support_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

      decaying_world_scan_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

   }

   public PointCloudWorldPacket(PointCloudWorldPacket other)
   {
      this();
      set(other);
   }

   public void set(PointCloudWorldPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      timestamp_ = other.timestamp_;

      ground_quad_tree_support_.set(other.ground_quad_tree_support_);
      decaying_world_scan_.set(other.decaying_world_scan_);
      default_ground_height_ = other.default_ground_height_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }

   public long getTimestamp()
   {
      return timestamp_;
   }

   public us.ihmc.idl.IDLSequence.Float getGroundQuadTreeSupport()
   {
      return ground_quad_tree_support_;
   }

   public us.ihmc.idl.IDLSequence.Float getDecayingWorldScan()
   {
      return decaying_world_scan_;
   }

   public void setDefaultGroundHeight(float default_ground_height)
   {
      default_ground_height_ = default_ground_height;
   }

   public float getDefaultGroundHeight()
   {
      return default_ground_height_;
   }

   @Override
   public boolean epsilonEquals(PointCloudWorldPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.ground_quad_tree_support_, other.ground_quad_tree_support_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.decaying_world_scan_, other.decaying_world_scan_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_ground_height_, other.default_ground_height_, epsilon))
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
      if (!(other instanceof PointCloudWorldPacket))
         return false;

      PointCloudWorldPacket otherMyClass = (PointCloudWorldPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.timestamp_ != otherMyClass.timestamp_)
         return false;

      if (!this.ground_quad_tree_support_.equals(otherMyClass.ground_quad_tree_support_))
         return false;
      if (!this.decaying_world_scan_.equals(otherMyClass.decaying_world_scan_))
         return false;
      if (this.default_ground_height_ != otherMyClass.default_ground_height_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PointCloudWorldPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);
      builder.append(", ");
      builder.append("ground_quad_tree_support=");
      builder.append(this.ground_quad_tree_support_);
      builder.append(", ");
      builder.append("decaying_world_scan=");
      builder.append(this.decaying_world_scan_);
      builder.append(", ");
      builder.append("default_ground_height=");
      builder.append(this.default_ground_height_);
      builder.append("}");
      return builder.toString();
   }
}
