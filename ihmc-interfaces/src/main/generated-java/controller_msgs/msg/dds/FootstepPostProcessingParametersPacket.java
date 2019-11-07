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
            * Determines whether the post processing module for adjusting the split fractions based on the footstep positions for the CoP trajectory is enabled.
            */
   public boolean position_split_fraction_processing_enabled_ = true;
   /**
            * Determines whether the post processing module for adjusting the split fractions based on the foothold areas for the CoP trajectory is enabled.
            */
   public boolean area_split_fraction_processing_enabled_ = true;
   /**
            * Determines whether the post processing module for swinging over planar regions is enabled.
            */
   public boolean swing_over_regions_processing_enabled_ = true;
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
   public boolean do_initial_fast_approximation_ = true;
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
   public double minimum_height_above_floor_for_collision_ = -1.0;
   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
            * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
            */
   public double fraction_load_if_foot_has_full_support_ = -1.0;
   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
            * from the foot to the midpoint, or from the midpoint to that foot.
            */
   public double fraction_time_on_foot_if_foot_has_full_support_ = -1.0;
   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
            * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
            * this movies the midpoint CoP position to that foot.
            */
   public double fraction_load_if_other_foot_has_no_width_ = -1.0;
   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
            * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
            */
   public double fraction_time_on_foot_if_other_foot_has_no_width_ = -1.0;

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

      position_split_fraction_processing_enabled_ = other.position_split_fraction_processing_enabled_;

      area_split_fraction_processing_enabled_ = other.area_split_fraction_processing_enabled_;

      swing_over_regions_processing_enabled_ = other.swing_over_regions_processing_enabled_;

      step_height_for_large_step_down_ = other.step_height_for_large_step_down_;

      largest_step_down_height_ = other.largest_step_down_height_;

      transfer_split_fraction_at_full_depth_ = other.transfer_split_fraction_at_full_depth_;

      transfer_weight_distribution_at_full_depth_ = other.transfer_weight_distribution_at_full_depth_;

      do_initial_fast_approximation_ = other.do_initial_fast_approximation_;

      minimum_swing_foot_clearance_ = other.minimum_swing_foot_clearance_;

      number_of_checks_per_swing_ = other.number_of_checks_per_swing_;

      maximum_number_of_adjustment_attempts_ = other.maximum_number_of_adjustment_attempts_;

      maximum_waypoint_adjustment_distance_ = other.maximum_waypoint_adjustment_distance_;

      incremental_waypoint_adjustment_distance_ = other.incremental_waypoint_adjustment_distance_;

      minimum_height_above_floor_for_collision_ = other.minimum_height_above_floor_for_collision_;

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
            * Determines whether the post processing module for adjusting the split fractions based on the footstep positions for the CoP trajectory is enabled.
            */
   public void setPositionSplitFractionProcessingEnabled(boolean position_split_fraction_processing_enabled)
   {
      position_split_fraction_processing_enabled_ = position_split_fraction_processing_enabled;
   }
   /**
            * Determines whether the post processing module for adjusting the split fractions based on the footstep positions for the CoP trajectory is enabled.
            */
   public boolean getPositionSplitFractionProcessingEnabled()
   {
      return position_split_fraction_processing_enabled_;
   }

   /**
            * Determines whether the post processing module for adjusting the split fractions based on the foothold areas for the CoP trajectory is enabled.
            */
   public void setAreaSplitFractionProcessingEnabled(boolean area_split_fraction_processing_enabled)
   {
      area_split_fraction_processing_enabled_ = area_split_fraction_processing_enabled;
   }
   /**
            * Determines whether the post processing module for adjusting the split fractions based on the foothold areas for the CoP trajectory is enabled.
            */
   public boolean getAreaSplitFractionProcessingEnabled()
   {
      return area_split_fraction_processing_enabled_;
   }

   /**
            * Determines whether the post processing module for swinging over planar regions is enabled.
            */
   public void setSwingOverRegionsProcessingEnabled(boolean swing_over_regions_processing_enabled)
   {
      swing_over_regions_processing_enabled_ = swing_over_regions_processing_enabled;
   }
   /**
            * Determines whether the post processing module for swinging over planar regions is enabled.
            */
   public boolean getSwingOverRegionsProcessingEnabled()
   {
      return swing_over_regions_processing_enabled_;
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

   public void setDoInitialFastApproximation(boolean do_initial_fast_approximation)
   {
      do_initial_fast_approximation_ = do_initial_fast_approximation;
   }
   public boolean getDoInitialFastApproximation()
   {
      return do_initial_fast_approximation_;
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

   public void setMinimumHeightAboveFloorForCollision(double minimum_height_above_floor_for_collision)
   {
      minimum_height_above_floor_for_collision_ = minimum_height_above_floor_for_collision;
   }
   public double getMinimumHeightAboveFloorForCollision()
   {
      return minimum_height_above_floor_for_collision_;
   }

   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
            * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
            */
   public void setFractionLoadIfFootHasFullSupport(double fraction_load_if_foot_has_full_support)
   {
      fraction_load_if_foot_has_full_support_ = fraction_load_if_foot_has_full_support;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if it has the full
            * support area. That is, if the foot has the full area, and we say it should carry the full load, this moves the midpoint CoP position to that foot.
            */
   public double getFractionLoadIfFootHasFullSupport()
   {
      return fraction_load_if_foot_has_full_support_;
   }

   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
            * from the foot to the midpoint, or from the midpoint to that foot.
            */
   public void setFractionTimeOnFootIfFootHasFullSupport(double fraction_time_on_foot_if_foot_has_full_support)
   {
      fraction_time_on_foot_if_foot_has_full_support_ = fraction_time_on_foot_if_foot_has_full_support;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if the foot has the full area, and we say it should have the entire trajectory (i.e. returns 1), this spends the entire time shifting either
            * from the foot to the midpoint, or from the midpoint to that foot.
            */
   public double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return fraction_time_on_foot_if_foot_has_full_support_;
   }

   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
            * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
            * this movies the midpoint CoP position to that foot.
            */
   public void setFractionLoadIfOtherFootHasNoWidth(double fraction_load_if_other_foot_has_no_width)
   {
      fraction_load_if_other_foot_has_no_width_ = fraction_load_if_other_foot_has_no_width;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the load a foot should carry during transfer if the trailing foot is
            * a forward line. That is, if there is only a line contact in the X direction on the other foot, and we say this foot should carry the full load,
            * this movies the midpoint CoP position to that foot.
            */
   public double getFractionLoadIfOtherFootHasNoWidth()
   {
      return fraction_load_if_other_foot_has_no_width_;
   }

   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
            * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
            */
   public void setFractionTimeOnFootIfOtherFootHasNoWidth(double fraction_time_on_foot_if_other_foot_has_no_width)
   {
      fraction_time_on_foot_if_other_foot_has_no_width_ = fraction_time_on_foot_if_other_foot_has_no_width;
   }
   /**
            * If using the area split fraction post processing module, this determines how much of the transfer duration should be spent shifting towards the midpoint
            * CoP. That is, if there is only a line contact in the X direction on the other foot, and we say it should have the entire trajectory (i.e. returns 1),
            * this spends the entire time shifting either from the foot to the midpoint, or from the midpoint to that foot.
            */
   public double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return fraction_time_on_foot_if_other_foot_has_no_width_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.position_split_fraction_processing_enabled_, other.position_split_fraction_processing_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.area_split_fraction_processing_enabled_, other.area_split_fraction_processing_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.swing_over_regions_processing_enabled_, other.swing_over_regions_processing_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_height_for_large_step_down_, other.step_height_for_large_step_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.largest_step_down_height_, other.largest_step_down_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_split_fraction_at_full_depth_, other.transfer_split_fraction_at_full_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_weight_distribution_at_full_depth_, other.transfer_weight_distribution_at_full_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.do_initial_fast_approximation_, other.do_initial_fast_approximation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_swing_foot_clearance_, other.minimum_swing_foot_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_checks_per_swing_, other.number_of_checks_per_swing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_number_of_adjustment_attempts_, other.maximum_number_of_adjustment_attempts_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_waypoint_adjustment_distance_, other.maximum_waypoint_adjustment_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incremental_waypoint_adjustment_distance_, other.incremental_waypoint_adjustment_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_height_above_floor_for_collision_, other.minimum_height_above_floor_for_collision_, epsilon)) return false;

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
      if(!(other instanceof FootstepPostProcessingParametersPacket)) return false;

      FootstepPostProcessingParametersPacket otherMyClass = (FootstepPostProcessingParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.position_split_fraction_processing_enabled_ != otherMyClass.position_split_fraction_processing_enabled_) return false;

      if(this.area_split_fraction_processing_enabled_ != otherMyClass.area_split_fraction_processing_enabled_) return false;

      if(this.swing_over_regions_processing_enabled_ != otherMyClass.swing_over_regions_processing_enabled_) return false;

      if(this.step_height_for_large_step_down_ != otherMyClass.step_height_for_large_step_down_) return false;

      if(this.largest_step_down_height_ != otherMyClass.largest_step_down_height_) return false;

      if(this.transfer_split_fraction_at_full_depth_ != otherMyClass.transfer_split_fraction_at_full_depth_) return false;

      if(this.transfer_weight_distribution_at_full_depth_ != otherMyClass.transfer_weight_distribution_at_full_depth_) return false;

      if(this.do_initial_fast_approximation_ != otherMyClass.do_initial_fast_approximation_) return false;

      if(this.minimum_swing_foot_clearance_ != otherMyClass.minimum_swing_foot_clearance_) return false;

      if(this.number_of_checks_per_swing_ != otherMyClass.number_of_checks_per_swing_) return false;

      if(this.maximum_number_of_adjustment_attempts_ != otherMyClass.maximum_number_of_adjustment_attempts_) return false;

      if(this.maximum_waypoint_adjustment_distance_ != otherMyClass.maximum_waypoint_adjustment_distance_) return false;

      if(this.incremental_waypoint_adjustment_distance_ != otherMyClass.incremental_waypoint_adjustment_distance_) return false;

      if(this.minimum_height_above_floor_for_collision_ != otherMyClass.minimum_height_above_floor_for_collision_) return false;

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

      builder.append("FootstepPostProcessingParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("position_split_fraction_processing_enabled=");
      builder.append(this.position_split_fraction_processing_enabled_);      builder.append(", ");
      builder.append("area_split_fraction_processing_enabled=");
      builder.append(this.area_split_fraction_processing_enabled_);      builder.append(", ");
      builder.append("swing_over_regions_processing_enabled=");
      builder.append(this.swing_over_regions_processing_enabled_);      builder.append(", ");
      builder.append("step_height_for_large_step_down=");
      builder.append(this.step_height_for_large_step_down_);      builder.append(", ");
      builder.append("largest_step_down_height=");
      builder.append(this.largest_step_down_height_);      builder.append(", ");
      builder.append("transfer_split_fraction_at_full_depth=");
      builder.append(this.transfer_split_fraction_at_full_depth_);      builder.append(", ");
      builder.append("transfer_weight_distribution_at_full_depth=");
      builder.append(this.transfer_weight_distribution_at_full_depth_);      builder.append(", ");
      builder.append("do_initial_fast_approximation=");
      builder.append(this.do_initial_fast_approximation_);      builder.append(", ");
      builder.append("minimum_swing_foot_clearance=");
      builder.append(this.minimum_swing_foot_clearance_);      builder.append(", ");
      builder.append("number_of_checks_per_swing=");
      builder.append(this.number_of_checks_per_swing_);      builder.append(", ");
      builder.append("maximum_number_of_adjustment_attempts=");
      builder.append(this.maximum_number_of_adjustment_attempts_);      builder.append(", ");
      builder.append("maximum_waypoint_adjustment_distance=");
      builder.append(this.maximum_waypoint_adjustment_distance_);      builder.append(", ");
      builder.append("incremental_waypoint_adjustment_distance=");
      builder.append(this.incremental_waypoint_adjustment_distance_);      builder.append(", ");
      builder.append("minimum_height_above_floor_for_collision=");
      builder.append(this.minimum_height_above_floor_for_collision_);      builder.append(", ");
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
