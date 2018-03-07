package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class FootstepList implements Settable<FootstepList>, EpsilonComparable<FootstepList>
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

   public FootstepList(FootstepList other)
   {
      set(other);
   }

   public void set(FootstepList other)
   {
      unique_id_ = other.unique_id_;

      footsteps_.set(other.footsteps_);
      executing_timing_ = other.executing_timing_;

      default_swing_duration_ = other.default_swing_duration_;

      final_transfer_duration_ = other.final_transfer_duration_;
   }

   public long getUniqueId()
   {
      return unique_id_;
   }

   public void setUniqueId(long unique_id)
   {
      unique_id_ = unique_id;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Footstep> getFootsteps()
   {
      return footsteps_;
   }

   public byte getExecutingTiming()
   {
      return executing_timing_;
   }

   public void setExecutingTiming(byte executing_timing)
   {
      executing_timing_ = executing_timing;
   }

   public double getDefaultSwingDuration()
   {
      return default_swing_duration_;
   }

   public void setDefaultSwingDuration(double default_swing_duration)
   {
      default_swing_duration_ = default_swing_duration;
   }

   public double getFinalTransferDuration()
   {
      return final_transfer_duration_;
   }

   public void setFinalTransferDuration(double final_transfer_duration)
   {
      final_transfer_duration_ = final_transfer_duration;
   }

   @Override
   public boolean epsilonEquals(FootstepList other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.unique_id_, other.unique_id_, epsilon))
         return false;

      if (this.footsteps_.isEnum())
      {
         if (!this.footsteps_.equals(other.footsteps_))
            return false;
      }
      else if (this.footsteps_.size() == other.footsteps_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.footsteps_.size(); i++)
         {
            if (!this.footsteps_.get(i).epsilonEquals(other.footsteps_.get(i), epsilon))
               return false;
         }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.executing_timing_, other.executing_timing_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_swing_duration_, other.default_swing_duration_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_transfer_duration_, other.final_transfer_duration_, epsilon))
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
      if (!(other instanceof FootstepList))
         return false;

      FootstepList otherMyClass = (FootstepList) other;

      if (this.unique_id_ != otherMyClass.unique_id_)
         return false;

      if (!this.footsteps_.equals(otherMyClass.footsteps_))
         return false;

      if (this.executing_timing_ != otherMyClass.executing_timing_)
         return false;

      if (this.default_swing_duration_ != otherMyClass.default_swing_duration_)
         return false;

      if (this.final_transfer_duration_ != otherMyClass.final_transfer_duration_)
         return false;

      return true;
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