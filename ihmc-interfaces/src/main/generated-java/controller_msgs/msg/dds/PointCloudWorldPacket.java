package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is obsolete
 */
public class PointCloudWorldPacket implements Settable<PointCloudWorldPacket>, EpsilonComparable<PointCloudWorldPacket>
{
   private long timestamp_;
   private us.ihmc.idl.IDLSequence.Float ground_quad_tree_support_;
   private us.ihmc.idl.IDLSequence.Float decaying_world_scan_;
   private float default_ground_height_;

   public PointCloudWorldPacket()
   {

      ground_quad_tree_support_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

      decaying_world_scan_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");
   }

   public PointCloudWorldPacket(PointCloudWorldPacket other)
   {
      set(other);
   }

   public void set(PointCloudWorldPacket other)
   {
      timestamp_ = other.timestamp_;

      ground_quad_tree_support_.set(other.ground_quad_tree_support_);
      decaying_world_scan_.set(other.decaying_world_scan_);
      default_ground_height_ = other.default_ground_height_;
   }

   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }

   public us.ihmc.idl.IDLSequence.Float getGroundQuadTreeSupport()
   {
      return ground_quad_tree_support_;
   }

   public us.ihmc.idl.IDLSequence.Float getDecayingWorldScan()
   {
      return decaying_world_scan_;
   }

   public float getDefaultGroundHeight()
   {
      return default_ground_height_;
   }

   public void setDefaultGroundHeight(float default_ground_height)
   {
      default_ground_height_ = default_ground_height;
   }

   @Override
   public boolean epsilonEquals(PointCloudWorldPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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