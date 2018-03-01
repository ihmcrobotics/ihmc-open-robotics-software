package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Definition of the class "PoseWithConfidenceStamped" defined in PoseWithConfidenceStamped_.idl.
 *
 * This file was automatically generated from PoseWithConfidenceStamped_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit PoseWithConfidenceStamped_.idl instead.
 */
public class PoseWithConfidenceStamped implements Settable<PoseWithConfidenceStamped>, EpsilonComparable<PoseWithConfidenceStamped>
{
   private geometry_msgs.msg.dds.PoseStamped pose_;
   private float confidence_;

   public PoseWithConfidenceStamped()
   {
      pose_ = new geometry_msgs.msg.dds.PoseStamped();
   }

   public PoseWithConfidenceStamped(PoseWithConfidenceStamped other)
   {
      set(other);
   }

   public void set(PoseWithConfidenceStamped other)
   {
      geometry_msgs.msg.dds.PoseStampedPubSubType.staticCopy(other.pose_, pose_);
      confidence_ = other.confidence_;
   }

   public geometry_msgs.msg.dds.PoseStamped getPose()
   {
      return pose_;
   }

   public float getConfidence()
   {
      return confidence_;
   }

   public void setConfidence(float confidence)
   {
      confidence_ = confidence;
   }

   @Override
   public boolean epsilonEquals(PoseWithConfidenceStamped other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_, other.confidence_, epsilon))
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
      if (!(other instanceof PoseWithConfidenceStamped))
         return false;

      PoseWithConfidenceStamped otherMyClass = (PoseWithConfidenceStamped) other;

      if (!this.pose_.equals(otherMyClass.pose_))
         return false;

      if (this.confidence_ != otherMyClass.confidence_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PoseWithConfidenceStamped {");
      builder.append("pose=");
      builder.append(this.pose_);

      builder.append(", ");
      builder.append("confidence=");
      builder.append(this.confidence_);

      builder.append("}");
      return builder.toString();
   }
}