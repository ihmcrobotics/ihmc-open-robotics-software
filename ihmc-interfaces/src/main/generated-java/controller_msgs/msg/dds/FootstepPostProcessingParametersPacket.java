package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPostProcessingParametersPacket extends Packet<FootstepPostProcessingParametersPacket> implements Settable<FootstepPostProcessingParametersPacket>, EpsilonComparable<FootstepPostProcessingParametersPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Determines whether the post processing module for adjusting the split fractions for the CoP trajectory is enabled.
            */
   public boolean split_fraction_processing_enabled_ = true;
   /**
            * Determines whether the post processing module for swinging over planar regions is enabled.
            */
   public boolean swing_over_regions_enabled_ = true;
   /**
            * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
            */
   public double step_height_for_large_step_down_ = -1.0;
   /**
            * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
            */
   public double largest_step_down_height_ = -1.0;
   /**
            * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired split fraction is linearly interpolated between the default value and the value returned by this function.
            */
   public double transfer_split_fraction_at_full_depth_ = -1.0;
   /**
            * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
            */
   public double transfer_weight_distribution_at_full_depth_ = -1.0;
   /**
            * If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
            * along the swing foot trajectory and the planar regions in the environment.
            */
   public double minimum_swing_foot_clearance_ = -1.0;
   /**
            * If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
            */
   public long number_of_checks_per_swing_ = 100;
   /**
            * If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
            * collisions with the environment.
            */
   public long maximum_number_of_adjustment_attempts_ = 50;
   /**
            * If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
            */
   public double maximum_waypoint_adjustment_distance_ = -1.0;
   /**
            * If using the swing over planar regions module, this is the distance that the swing waypoints will be adjusted by.
            */
   public double incremental_waypoint_adjustment_distance_ = -1.0;

   public FootstepPostProcessingParametersPacket()
   {
   }

   public FootstepPostProcessingParametersPacket(FootstepPostProcessingParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPostProcessingParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      split_fraction_processing_enabled_ = other.split_fraction_processing_enabled_;

      swing_over_regions_enabled_ = other.swing_over_regions_enabled_;

      step_height_for_large_step_down_ = other.step_height_for_large_step_down_;

      largest_step_down_height_ = other.largest_step_down_height_;

      transfer_split_fraction_at_full_depth_ = other.transfer_split_fraction_at_full_depth_;

      transfer_weight_distribution_at_full_depth_ = other.transfer_weight_distribution_at_full_depth_;

      minimum_swing_foot_clearance_ = other.minimum_swing_foot_clearance_;

      number_of_checks_per_swing_ = other.number_of_checks_per_swing_;

      maximum_number_of_adjustment_attempts_ = other.maximum_number_of_adjustment_attempts_;

      maximum_waypoint_adjustment_distance_ = other.maximum_waypoint_adjustment_distance_;

      incremental_waypoint_adjustment_distance_ = other.incremental_waypoint_adjustment_distance_;

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
            * Determines whether the post processing module for adjusting the split fractions for the CoP trajectory is enabled.
            */
   public void setSplitFractionProcessingEnabled(boolean split_fraction_processing_enabled)
   {
      split_fraction_processing_enabled_ = split_fraction_processing_enabled;
   }
   /**
            * Determines whether the post processing module for adjusting the split fractions for the CoP trajectory is enabled.
            */
   public boolean getSplitFractionProcessingEnabled()
   {
      return split_fraction_processing_enabled_;
   }

   /**
            * Determines whether the post processing module for swinging over planar regions is enabled.
            */
   public void setSwingOverRegionsEnabled(boolean swing_over_regions_enabled)
   {
      swing_over_regions_enabled_ = swing_over_regions_enabled;
   }
   /**
            * Determines whether the post processing module for swinging over planar regions is enabled.
            */
   public boolean getSwingOverRegionsEnabled()
   {
      return swing_over_regions_enabled_;
   }

   /**
            * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
            */
   public void setStepHeightForLargeStepDown(double step_height_for_large_step_down)
   {
      step_height_for_large_step_down_ = step_height_for_large_step_down;
   }
   /**
            * Sets the step down height for determining whether or not the transfer split fractions should be adjusted.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted so that the CoM is in a more favorable position, kind of "dropping" onto the swing foot.
            */
   public double getStepHeightForLargeStepDown()
   {
      return step_height_for_large_step_down_;
   }

   /**
            * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
            */
   public void setLargestStepDownHeight(double largest_step_down_height)
   {
      largest_step_down_height_ = largest_step_down_height;
   }
   /**
            * Sets the step down height for the maximum amount of split fraction and weight distribution adjustment.
            * If the step height change relative to the stance foot is greater than this value, the split fraction and weight distribution
            * will be adjusted fully, as returned by {@link #getTransferSplitFractionAtFullDepth()} and {@link #getTransferWeightDistributionAtFullDepth()}.
            */
   public double getLargestStepDownHeight()
   {
      return largest_step_down_height_;
   }

   /**
            * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired split fraction is linearly interpolated between the default value and the value returned by this function.
            */
   public void setTransferSplitFractionAtFullDepth(double transfer_split_fraction_at_full_depth)
   {
      transfer_split_fraction_at_full_depth_ = transfer_split_fraction_at_full_depth;
   }
   /**
            * Sets the desired transfer split fraction if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired split fraction is linearly interpolated between the default value and the value returned by this function.
            */
   public double getTransferSplitFractionAtFullDepth()
   {
      return transfer_split_fraction_at_full_depth_;
   }

   /**
            * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
            */
   public void setTransferWeightDistributionAtFullDepth(double transfer_weight_distribution_at_full_depth)
   {
      transfer_weight_distribution_at_full_depth_ = transfer_weight_distribution_at_full_depth;
   }
   /**
            * Sets the desired transfer weight distribution if the robot is stepping down by {@link #getLargestStepDownHeight()}.
            * If the step down height is between {@link #getStepHeightForLargeStepDown()} and {@link #getLargestStepDownHeight()}, the
            * desired weight distribution is linearly interpolated between the default value and the value returned by this function.
            */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return transfer_weight_distribution_at_full_depth_;
   }

   /**
            * If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
            * along the swing foot trajectory and the planar regions in the environment.
            */
   public void setMinimumSwingFootClearance(double minimum_swing_foot_clearance)
   {
      minimum_swing_foot_clearance_ = minimum_swing_foot_clearance;
   }
   /**
            * If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
            * along the swing foot trajectory and the planar regions in the environment.
            */
   public double getMinimumSwingFootClearance()
   {
      return minimum_swing_foot_clearance_;
   }

   /**
            * If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
            */
   public void setNumberOfChecksPerSwing(long number_of_checks_per_swing)
   {
      number_of_checks_per_swing_ = number_of_checks_per_swing;
   }
   /**
            * If using the swing over planar regions module, this is the number of points along the swing foot trajectory that are checked.
            */
   public long getNumberOfChecksPerSwing()
   {
      return number_of_checks_per_swing_;
   }

   /**
            * If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
            * collisions with the environment.
            */
   public void setMaximumNumberOfAdjustmentAttempts(long maximum_number_of_adjustment_attempts)
   {
      maximum_number_of_adjustment_attempts_ = maximum_number_of_adjustment_attempts;
   }
   /**
            * If using the swing over planar regions module, this is the maximum number of iterations for adjusting the swing foot waypoints to attempt avoiding
            * collisions with the environment.
            */
   public long getMaximumNumberOfAdjustmentAttempts()
   {
      return maximum_number_of_adjustment_attempts_;
   }

   /**
            * If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
            */
   public void setMaximumWaypointAdjustmentDistance(double maximum_waypoint_adjustment_distance)
   {
      maximum_waypoint_adjustment_distance_ = maximum_waypoint_adjustment_distance;
   }
   /**
            * If using the swing over planar regions module, this is the maximum adjustment distance of the swing waypoints that will be allowed.
            */
   public double getMaximumWaypointAdjustmentDistance()
   {
      return maximum_waypoint_adjustment_distance_;
   }

   /**
            * If using the swing over planar regions module, this is the distance that the swing waypoints will be adjusted by.
            */
   public void setIncrementalWaypointAdjustmentDistance(double incremental_waypoint_adjustment_distance)
   {
      incremental_waypoint_adjustment_distance_ = incremental_waypoint_adjustment_distance;
   }
   /**
            * If using the swing over planar regions module, this is the distance that the swing waypoints will be adjusted by.
            */
   public double getIncrementalWaypointAdjustmentDistance()
   {
      return incremental_waypoint_adjustment_distance_;
   }


   public static Supplier<FootstepPostProcessingParametersPacketPubSubType> getPubSubType()
   {
      return FootstepPostProcessingParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPostProcessingParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPostProcessingParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.split_fraction_processing_enabled_, other.split_fraction_processing_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.swing_over_regions_enabled_, other.swing_over_regions_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_height_for_large_step_down_, other.step_height_for_large_step_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.largest_step_down_height_, other.largest_step_down_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_split_fraction_at_full_depth_, other.transfer_split_fraction_at_full_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_weight_distribution_at_full_depth_, other.transfer_weight_distribution_at_full_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_swing_foot_clearance_, other.minimum_swing_foot_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_checks_per_swing_, other.number_of_checks_per_swing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_number_of_adjustment_attempts_, other.maximum_number_of_adjustment_attempts_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_waypoint_adjustment_distance_, other.maximum_waypoint_adjustment_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incremental_waypoint_adjustment_distance_, other.incremental_waypoint_adjustment_distance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPostProcessingParametersPacket)) return false;

      FootstepPostProcessingParametersPacket otherMyClass = (FootstepPostProcessingParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.split_fraction_processing_enabled_ != otherMyClass.split_fraction_processing_enabled_) return false;

      if(this.swing_over_regions_enabled_ != otherMyClass.swing_over_regions_enabled_) return false;

      if(this.step_height_for_large_step_down_ != otherMyClass.step_height_for_large_step_down_) return false;

      if(this.largest_step_down_height_ != otherMyClass.largest_step_down_height_) return false;

      if(this.transfer_split_fraction_at_full_depth_ != otherMyClass.transfer_split_fraction_at_full_depth_) return false;

      if(this.transfer_weight_distribution_at_full_depth_ != otherMyClass.transfer_weight_distribution_at_full_depth_) return false;

      if(this.minimum_swing_foot_clearance_ != otherMyClass.minimum_swing_foot_clearance_) return false;

      if(this.number_of_checks_per_swing_ != otherMyClass.number_of_checks_per_swing_) return false;

      if(this.maximum_number_of_adjustment_attempts_ != otherMyClass.maximum_number_of_adjustment_attempts_) return false;

      if(this.maximum_waypoint_adjustment_distance_ != otherMyClass.maximum_waypoint_adjustment_distance_) return false;

      if(this.incremental_waypoint_adjustment_distance_ != otherMyClass.incremental_waypoint_adjustment_distance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPostProcessingParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("split_fraction_processing_enabled=");
      builder.append(this.split_fraction_processing_enabled_);      builder.append(", ");
      builder.append("swing_over_regions_enabled=");
      builder.append(this.swing_over_regions_enabled_);      builder.append(", ");
      builder.append("step_height_for_large_step_down=");
      builder.append(this.step_height_for_large_step_down_);      builder.append(", ");
      builder.append("largest_step_down_height=");
      builder.append(this.largest_step_down_height_);      builder.append(", ");
      builder.append("transfer_split_fraction_at_full_depth=");
      builder.append(this.transfer_split_fraction_at_full_depth_);      builder.append(", ");
      builder.append("transfer_weight_distribution_at_full_depth=");
      builder.append(this.transfer_weight_distribution_at_full_depth_);      builder.append(", ");
      builder.append("minimum_swing_foot_clearance=");
      builder.append(this.minimum_swing_foot_clearance_);      builder.append(", ");
      builder.append("number_of_checks_per_swing=");
      builder.append(this.number_of_checks_per_swing_);      builder.append(", ");
      builder.append("maximum_number_of_adjustment_attempts=");
      builder.append(this.maximum_number_of_adjustment_attempts_);      builder.append(", ");
      builder.append("maximum_waypoint_adjustment_distance=");
      builder.append(this.maximum_waypoint_adjustment_distance_);      builder.append(", ");
      builder.append("incremental_waypoint_adjustment_distance=");
      builder.append(this.incremental_waypoint_adjustment_distance_);
      builder.append("}");
      return builder.toString();
   }
}
