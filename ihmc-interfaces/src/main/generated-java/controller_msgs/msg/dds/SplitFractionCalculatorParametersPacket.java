package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class SplitFractionCalculatorParametersPacket extends Packet<SplitFractionCalculatorParametersPacket> implements Settable<SplitFractionCalculatorParametersPacket>, EpsilonComparable<SplitFractionCalculatorParametersPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Default value for the ICP plan's transfer split fraction.
            */
   public double default_transfer_split_fraction_ = -1.0;

   /**
            * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
            * Field default value -1.0
            */
   public double step_height_for_large_step_down_;

   /**
            * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
            * Field default value -1.0
            */
   public double largest_step_down_height_;

   /**
            * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired split fraction is linearly interpolated between the default value and the value returned by this function.
            * Field default value -1.0
            */
   public double transfer_split_fraction_at_full_depth_;

   /**
            * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
            * Field default value -1.0
            */
   public double transfer_weight_distribution_at_full_depth_;

   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
            * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
            * Field default value -1.0
            */
   public double fraction_load_if_foot_has_full_support_;

   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
            * from the foot to the midpoint, or from the midpoint to that foot.
            * Field default value -1.0
            */
   public double fraction_time_on_foot_if_foot_has_full_support_;

   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
            * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
            * this movies the midpoint CoP position to that foot.
            * Field default value -1.0
            */
   public double fraction_load_if_other_foot_has_no_width_;

   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
            * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
            * Field default value -1.0
            */
   public double fraction_time_on_foot_if_other_foot_has_no_width_;

   public SplitFractionCalculatorParametersPacket()
   {











   }

   public SplitFractionCalculatorParametersPacket(SplitFractionCalculatorParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(SplitFractionCalculatorParametersPacket other)
   {

      sequence_id_ = other.sequence_id_;


      default_transfer_split_fraction_ = other.default_transfer_split_fraction_;


      step_height_for_large_step_down_ = other.step_height_for_large_step_down_;


      largest_step_down_height_ = other.largest_step_down_height_;


      transfer_split_fraction_at_full_depth_ = other.transfer_split_fraction_at_full_depth_;


      transfer_weight_distribution_at_full_depth_ = other.transfer_weight_distribution_at_full_depth_;


      fraction_load_if_foot_has_full_support_ = other.fraction_load_if_foot_has_full_support_;


      fraction_time_on_foot_if_foot_has_full_support_ = other.fraction_time_on_foot_if_foot_has_full_support_;


      fraction_load_if_other_foot_has_no_width_ = other.fraction_load_if_other_foot_has_no_width_;


      fraction_time_on_foot_if_other_foot_has_no_width_ = other.fraction_time_on_foot_if_other_foot_has_no_width_;

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   /**
            * Default value for the ICP plan's transfer split fraction.
            */
   public void setDefaultTransferSplitFraction(double default_transfer_split_fraction)
   {
      default_transfer_split_fraction_ = default_transfer_split_fraction;
   }
   /**
            * Default value for the ICP plan's transfer split fraction.
            */
   public double getDefaultTransferSplitFraction()
   {
      return default_transfer_split_fraction_;
   }


   /**
            * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
            * Field default value -1.0
            */
   public void setStepHeightForLargeStepDown(double step_height_for_large_step_down)
   {
      step_height_for_large_step_down_ = step_height_for_large_step_down;
   }
   /**
            * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
            * Field default value -1.0
            */
   public double getStepHeightForLargeStepDown()
   {
      return step_height_for_large_step_down_;
   }


   /**
            * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
            * Field default value -1.0
            */
   public void setLargestStepDownHeight(double largest_step_down_height)
   {
      largest_step_down_height_ = largest_step_down_height;
   }
   /**
            * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
            * Field default value -1.0
            */
   public double getLargestStepDownHeight()
   {
      return largest_step_down_height_;
   }


   /**
            * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired split fraction is linearly interpolated between the default value and the value returned by this function.
            * Field default value -1.0
            */
   public void setTransferSplitFractionAtFullDepth(double transfer_split_fraction_at_full_depth)
   {
      transfer_split_fraction_at_full_depth_ = transfer_split_fraction_at_full_depth;
   }
   /**
            * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired split fraction is linearly interpolated between the default value and the value returned by this function.
            * Field default value -1.0
            */
   public double getTransferSplitFractionAtFullDepth()
   {
      return transfer_split_fraction_at_full_depth_;
   }


   /**
            * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
            * Field default value -1.0
            */
   public void setTransferWeightDistributionAtFullDepth(double transfer_weight_distribution_at_full_depth)
   {
      transfer_weight_distribution_at_full_depth_ = transfer_weight_distribution_at_full_depth;
   }
   /**
            * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
            * Field default value -1.0
            */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return transfer_weight_distribution_at_full_depth_;
   }


   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
            * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
            * Field default value -1.0
            */
   public void setFractionLoadIfFootHasFullSupport(double fraction_load_if_foot_has_full_support)
   {
      fraction_load_if_foot_has_full_support_ = fraction_load_if_foot_has_full_support;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
            * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
            * Field default value -1.0
            */
   public double getFractionLoadIfFootHasFullSupport()
   {
      return fraction_load_if_foot_has_full_support_;
   }


   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
            * from the foot to the midpoint, or from the midpoint to that foot.
            * Field default value -1.0
            */
   public void setFractionTimeOnFootIfFootHasFullSupport(double fraction_time_on_foot_if_foot_has_full_support)
   {
      fraction_time_on_foot_if_foot_has_full_support_ = fraction_time_on_foot_if_foot_has_full_support;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
            * from the foot to the midpoint, or from the midpoint to that foot.
            * Field default value -1.0
            */
   public double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return fraction_time_on_foot_if_foot_has_full_support_;
   }


   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
            * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
            * this movies the midpoint CoP position to that foot.
            * Field default value -1.0
            */
   public void setFractionLoadIfOtherFootHasNoWidth(double fraction_load_if_other_foot_has_no_width)
   {
      fraction_load_if_other_foot_has_no_width_ = fraction_load_if_other_foot_has_no_width;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
            * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
            * this movies the midpoint CoP position to that foot.
            * Field default value -1.0
            */
   public double getFractionLoadIfOtherFootHasNoWidth()
   {
      return fraction_load_if_other_foot_has_no_width_;
   }


   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
            * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
            * Field default value -1.0
            */
   public void setFractionTimeOnFootIfOtherFootHasNoWidth(double fraction_time_on_foot_if_other_foot_has_no_width)
   {
      fraction_time_on_foot_if_other_foot_has_no_width_ = fraction_time_on_foot_if_other_foot_has_no_width;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
            * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
            * Field default value -1.0
            */
   public double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return fraction_time_on_foot_if_other_foot_has_no_width_;
   }


   public static Supplier<SplitFractionCalculatorParametersPacketPubSubType> getPubSubType()
   {
      return SplitFractionCalculatorParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SplitFractionCalculatorParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SplitFractionCalculatorParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_transfer_split_fraction_, other.default_transfer_split_fraction_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_height_for_large_step_down_, other.step_height_for_large_step_down_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.largest_step_down_height_, other.largest_step_down_height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_split_fraction_at_full_depth_, other.transfer_split_fraction_at_full_depth_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_weight_distribution_at_full_depth_, other.transfer_weight_distribution_at_full_depth_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fraction_load_if_foot_has_full_support_, other.fraction_load_if_foot_has_full_support_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fraction_time_on_foot_if_foot_has_full_support_, other.fraction_time_on_foot_if_foot_has_full_support_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fraction_load_if_other_foot_has_no_width_, other.fraction_load_if_other_foot_has_no_width_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fraction_time_on_foot_if_other_foot_has_no_width_, other.fraction_time_on_foot_if_other_foot_has_no_width_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SplitFractionCalculatorParametersPacket)) return false;

      SplitFractionCalculatorParametersPacket otherMyClass = (SplitFractionCalculatorParametersPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.default_transfer_split_fraction_ != otherMyClass.default_transfer_split_fraction_) return false;


      if(this.step_height_for_large_step_down_ != otherMyClass.step_height_for_large_step_down_) return false;


      if(this.largest_step_down_height_ != otherMyClass.largest_step_down_height_) return false;


      if(this.transfer_split_fraction_at_full_depth_ != otherMyClass.transfer_split_fraction_at_full_depth_) return false;


      if(this.transfer_weight_distribution_at_full_depth_ != otherMyClass.transfer_weight_distribution_at_full_depth_) return false;


      if(this.fraction_load_if_foot_has_full_support_ != otherMyClass.fraction_load_if_foot_has_full_support_) return false;


      if(this.fraction_time_on_foot_if_foot_has_full_support_ != otherMyClass.fraction_time_on_foot_if_foot_has_full_support_) return false;


      if(this.fraction_load_if_other_foot_has_no_width_ != otherMyClass.fraction_load_if_other_foot_has_no_width_) return false;


      if(this.fraction_time_on_foot_if_other_foot_has_no_width_ != otherMyClass.fraction_time_on_foot_if_other_foot_has_no_width_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SplitFractionCalculatorParametersPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("default_transfer_split_fraction=");
      builder.append(this.default_transfer_split_fraction_);      builder.append(", ");

      builder.append("step_height_for_large_step_down=");
      builder.append(this.step_height_for_large_step_down_);      builder.append(", ");

      builder.append("largest_step_down_height=");
      builder.append(this.largest_step_down_height_);      builder.append(", ");

      builder.append("transfer_split_fraction_at_full_depth=");
      builder.append(this.transfer_split_fraction_at_full_depth_);      builder.append(", ");

      builder.append("transfer_weight_distribution_at_full_depth=");
      builder.append(this.transfer_weight_distribution_at_full_depth_);      builder.append(", ");

      builder.append("fraction_load_if_foot_has_full_support=");
      builder.append(this.fraction_load_if_foot_has_full_support_);      builder.append(", ");

      builder.append("fraction_time_on_foot_if_foot_has_full_support=");
      builder.append(this.fraction_time_on_foot_if_foot_has_full_support_);      builder.append(", ");

      builder.append("fraction_load_if_other_foot_has_no_width=");
      builder.append(this.fraction_load_if_other_foot_has_no_width_);      builder.append(", ");

      builder.append("fraction_time_on_foot_if_other_foot_has_no_width=");
      builder.append(this.fraction_time_on_foot_if_other_foot_has_no_width_);
      builder.append("}");
      return builder.toString();
   }
}
