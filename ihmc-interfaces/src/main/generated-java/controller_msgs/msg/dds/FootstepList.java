package controller_msgs.msg.dds;

/**
 * Definition of the class "FootstepList" defined in FootstepList_.idl.
 *
 * This file was automatically generated from FootstepList_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit FootstepList_.idl instead.
 */
public class FootstepList
{
   private long unique_id_;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Footstep> footsteps_;
   private byte executing_timing_;
   private double default_swing_duration_;
   private double final_transfer_duration_;

   public FootstepList()
   {
      footsteps_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Footstep>(100, controller_msgs.msg.dds.Footstep.class,
                                                                                        new controller_msgs.msg.dds.FootstepPubSubType());
   }

   public void set(FootstepList other)
   {
      unique_id_ = other.unique_id_;
      footsteps_.set(other.footsteps_);
      executing_timing_ = other.executing_timing_;
      default_swing_duration_ = other.default_swing_duration_;
      final_transfer_duration_ = other.final_transfer_duration_;
   }

   public long getUnique_id()
   {
      return unique_id_;
   }

   public void setUnique_id(long unique_id)
   {
      unique_id_ = unique_id;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Footstep> getFootsteps()
   {
      return footsteps_;
   }

   public byte getExecuting_timing()
   {
      return executing_timing_;
   }

   public void setExecuting_timing(byte executing_timing)
   {
      executing_timing_ = executing_timing;
   }

   public double getDefault_swing_duration()
   {
      return default_swing_duration_;
   }

   public void setDefault_swing_duration(double default_swing_duration)
   {
      default_swing_duration_ = default_swing_duration;
   }

   public double getFinal_transfer_duration()
   {
      return final_transfer_duration_;
   }

   public void setFinal_transfer_duration(double final_transfer_duration)
   {
      final_transfer_duration_ = final_transfer_duration;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof FootstepList))
         return false;
      FootstepList otherMyClass = (FootstepList) other;
      boolean returnedValue = true;

      returnedValue &= this.unique_id_ == otherMyClass.unique_id_;

      returnedValue &= this.footsteps_.equals(otherMyClass.footsteps_);

      returnedValue &= this.executing_timing_ == otherMyClass.executing_timing_;

      returnedValue &= this.default_swing_duration_ == otherMyClass.default_swing_duration_;

      returnedValue &= this.final_transfer_duration_ == otherMyClass.final_transfer_duration_;

      return returnedValue;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepList {");
      builder.append("unique_id=");
      builder.append(this.unique_id_);

      builder.append(", ");
      builder.append("footsteps=");
      builder.append(this.footsteps_);

      builder.append(", ");
      builder.append("executing_timing=");
      builder.append(this.executing_timing_);

      builder.append(", ");
      builder.append("default_swing_duration=");
      builder.append(this.default_swing_duration_);

      builder.append(", ");
      builder.append("final_transfer_duration=");
      builder.append(this.final_transfer_duration_);

      builder.append("}");
      return builder.toString();
   }
}