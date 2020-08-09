package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class SwingPlannerParametersPacket extends Packet<SwingPlannerParametersPacket> implements Settable<SwingPlannerParametersPacket>, EpsilonComparable<SwingPlannerParametersPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Field default value True
            */
   public boolean do_initial_fast_approximation_ = true;

   /**
            * If using the swing over planar regions module, this sets up the minimum swing foot clearance distance between the a ball of radius of the foot length
            * along the swing foot trajectory and the planar regions in the environment.
            */
   public double minimum_swing_foot_clearance_ = -1.0;

   /**
            * If using the swing over planar regions module, this sets up how much less clearance is required on the fast approximation, since it doesn't usually have
            * the same amount of curve to the trajectory.
            */
   public double fast_approximation_less_clearance_ = -1.0;

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
            * If using the swing over planar regions module, this is the minimum distance that the swing waypoints will be adjusted by on each increment.
            */
   public double minimum_adjustment_increment_distance_ = -1.0;

   /**
            * If using the swing over planar regions module, this is the maximum distance that the swing waypoints will be adjusted by on each increment.
            */
   public double maximum_adjustment_increment_distance_ = -1.0;

   /**
            * If using the swing over planar regions module, this is the scale factor to be applied to the collision on each increment for adjustment.
            */
   public double adjustment_increment_distance_gain_ = -1.0;

   public double minimum_height_above_floor_for_collision_ = -1.0;

   /**
            * Collision boxes are checked for the start and end of swing at the toe and heel, respectively.
            * If a collision is detected at either, this swing height is used.
            */
   public double swing_height_if_collision_detected_ = -1.0;

   /**
            * Specifies the maximum swing time in the swing waypoint proportion calculator
            */
   public double minimum_swing_time_ = -1.0;

   /**
            * Specifies the maximum swing time in the swing waypoint proportion calculator
            */
   public double maximum_swing_time_ = -1.0;

   /**
            * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies the clearance of that collision
            */
   public double foot_stub_clearance_ = -1.0;

   /**
            * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies how much to shift if a collision is detected
            */
   public double waypoint_proportion_shift_for_stub_avoidance_ = -1.0;

   /**
            * If using the custom position swing planner, will add additional swing time if the trajectory is expanded
            */
   public double additional_swing_time_if_expanded_ = -1.0;

   public SwingPlannerParametersPacket()
   {


















   }

   public SwingPlannerParametersPacket(SwingPlannerParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(SwingPlannerParametersPacket other)
   {

      sequence_id_ = other.sequence_id_;


      do_initial_fast_approximation_ = other.do_initial_fast_approximation_;


      minimum_swing_foot_clearance_ = other.minimum_swing_foot_clearance_;


      fast_approximation_less_clearance_ = other.fast_approximation_less_clearance_;


      number_of_checks_per_swing_ = other.number_of_checks_per_swing_;


      maximum_number_of_adjustment_attempts_ = other.maximum_number_of_adjustment_attempts_;


      maximum_waypoint_adjustment_distance_ = other.maximum_waypoint_adjustment_distance_;


      minimum_adjustment_increment_distance_ = other.minimum_adjustment_increment_distance_;


      maximum_adjustment_increment_distance_ = other.maximum_adjustment_increment_distance_;


      adjustment_increment_distance_gain_ = other.adjustment_increment_distance_gain_;


      minimum_height_above_floor_for_collision_ = other.minimum_height_above_floor_for_collision_;


      swing_height_if_collision_detected_ = other.swing_height_if_collision_detected_;


      minimum_swing_time_ = other.minimum_swing_time_;


      maximum_swing_time_ = other.maximum_swing_time_;


      foot_stub_clearance_ = other.foot_stub_clearance_;


      waypoint_proportion_shift_for_stub_avoidance_ = other.waypoint_proportion_shift_for_stub_avoidance_;


      additional_swing_time_if_expanded_ = other.additional_swing_time_if_expanded_;

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
            * Field default value True
            */
   public void setDoInitialFastApproximation(boolean do_initial_fast_approximation)
   {
      do_initial_fast_approximation_ = do_initial_fast_approximation;
   }
   /**
            * Field default value True
            */
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
            * If using the swing over planar regions module, this sets up how much less clearance is required on the fast approximation, since it doesn't usually have
            * the same amount of curve to the trajectory.
            */
   public void setFastApproximationLessClearance(double fast_approximation_less_clearance)
   {
      fast_approximation_less_clearance_ = fast_approximation_less_clearance;
   }
   /**
            * If using the swing over planar regions module, this sets up how much less clearance is required on the fast approximation, since it doesn't usually have
            * the same amount of curve to the trajectory.
            */
   public double getFastApproximationLessClearance()
   {
      return fast_approximation_less_clearance_;
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
            * If using the swing over planar regions module, this is the minimum distance that the swing waypoints will be adjusted by on each increment.
            */
   public void setMinimumAdjustmentIncrementDistance(double minimum_adjustment_increment_distance)
   {
      minimum_adjustment_increment_distance_ = minimum_adjustment_increment_distance;
   }
   /**
            * If using the swing over planar regions module, this is the minimum distance that the swing waypoints will be adjusted by on each increment.
            */
   public double getMinimumAdjustmentIncrementDistance()
   {
      return minimum_adjustment_increment_distance_;
   }


   /**
            * If using the swing over planar regions module, this is the maximum distance that the swing waypoints will be adjusted by on each increment.
            */
   public void setMaximumAdjustmentIncrementDistance(double maximum_adjustment_increment_distance)
   {
      maximum_adjustment_increment_distance_ = maximum_adjustment_increment_distance;
   }
   /**
            * If using the swing over planar regions module, this is the maximum distance that the swing waypoints will be adjusted by on each increment.
            */
   public double getMaximumAdjustmentIncrementDistance()
   {
      return maximum_adjustment_increment_distance_;
   }


   /**
            * If using the swing over planar regions module, this is the scale factor to be applied to the collision on each increment for adjustment.
            */
   public void setAdjustmentIncrementDistanceGain(double adjustment_increment_distance_gain)
   {
      adjustment_increment_distance_gain_ = adjustment_increment_distance_gain;
   }
   /**
            * If using the swing over planar regions module, this is the scale factor to be applied to the collision on each increment for adjustment.
            */
   public double getAdjustmentIncrementDistanceGain()
   {
      return adjustment_increment_distance_gain_;
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
            * Collision boxes are checked for the start and end of swing at the toe and heel, respectively.
            * If a collision is detected at either, this swing height is used.
            */
   public void setSwingHeightIfCollisionDetected(double swing_height_if_collision_detected)
   {
      swing_height_if_collision_detected_ = swing_height_if_collision_detected;
   }
   /**
            * Collision boxes are checked for the start and end of swing at the toe and heel, respectively.
            * If a collision is detected at either, this swing height is used.
            */
   public double getSwingHeightIfCollisionDetected()
   {
      return swing_height_if_collision_detected_;
   }


   /**
            * Specifies the maximum swing time in the swing waypoint proportion calculator
            */
   public void setMinimumSwingTime(double minimum_swing_time)
   {
      minimum_swing_time_ = minimum_swing_time;
   }
   /**
            * Specifies the maximum swing time in the swing waypoint proportion calculator
            */
   public double getMinimumSwingTime()
   {
      return minimum_swing_time_;
   }


   /**
            * Specifies the maximum swing time in the swing waypoint proportion calculator
            */
   public void setMaximumSwingTime(double maximum_swing_time)
   {
      maximum_swing_time_ = maximum_swing_time;
   }
   /**
            * Specifies the maximum swing time in the swing waypoint proportion calculator
            */
   public double getMaximumSwingTime()
   {
      return maximum_swing_time_;
   }


   /**
            * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies the clearance of that collision
            */
   public void setFootStubClearance(double foot_stub_clearance)
   {
      foot_stub_clearance_ = foot_stub_clearance;
   }
   /**
            * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies the clearance of that collision
            */
   public double getFootStubClearance()
   {
      return foot_stub_clearance_;
   }


   /**
            * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies how much to shift if a collision is detected
            */
   public void setWaypointProportionShiftForStubAvoidance(double waypoint_proportion_shift_for_stub_avoidance)
   {
      waypoint_proportion_shift_for_stub_avoidance_ = waypoint_proportion_shift_for_stub_avoidance;
   }
   /**
            * The waypoint proportion calculator shifts the specified proportions if a foot collision is detected. This specifies how much to shift if a collision is detected
            */
   public double getWaypointProportionShiftForStubAvoidance()
   {
      return waypoint_proportion_shift_for_stub_avoidance_;
   }


   /**
            * If using the custom position swing planner, will add additional swing time if the trajectory is expanded
            */
   public void setAdditionalSwingTimeIfExpanded(double additional_swing_time_if_expanded)
   {
      additional_swing_time_if_expanded_ = additional_swing_time_if_expanded;
   }
   /**
            * If using the custom position swing planner, will add additional swing time if the trajectory is expanded
            */
   public double getAdditionalSwingTimeIfExpanded()
   {
      return additional_swing_time_if_expanded_;
   }


   public static Supplier<SwingPlannerParametersPacketPubSubType> getPubSubType()
   {
      return SwingPlannerParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SwingPlannerParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SwingPlannerParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.do_initial_fast_approximation_, other.do_initial_fast_approximation_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_swing_foot_clearance_, other.minimum_swing_foot_clearance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fast_approximation_less_clearance_, other.fast_approximation_less_clearance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_checks_per_swing_, other.number_of_checks_per_swing_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_number_of_adjustment_attempts_, other.maximum_number_of_adjustment_attempts_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_waypoint_adjustment_distance_, other.maximum_waypoint_adjustment_distance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_adjustment_increment_distance_, other.minimum_adjustment_increment_distance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_adjustment_increment_distance_, other.maximum_adjustment_increment_distance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.adjustment_increment_distance_gain_, other.adjustment_increment_distance_gain_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_height_above_floor_for_collision_, other.minimum_height_above_floor_for_collision_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_if_collision_detected_, other.swing_height_if_collision_detected_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_swing_time_, other.minimum_swing_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_swing_time_, other.maximum_swing_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_stub_clearance_, other.foot_stub_clearance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.waypoint_proportion_shift_for_stub_avoidance_, other.waypoint_proportion_shift_for_stub_avoidance_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.additional_swing_time_if_expanded_, other.additional_swing_time_if_expanded_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SwingPlannerParametersPacket)) return false;

      SwingPlannerParametersPacket otherMyClass = (SwingPlannerParametersPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.do_initial_fast_approximation_ != otherMyClass.do_initial_fast_approximation_) return false;


      if(this.minimum_swing_foot_clearance_ != otherMyClass.minimum_swing_foot_clearance_) return false;


      if(this.fast_approximation_less_clearance_ != otherMyClass.fast_approximation_less_clearance_) return false;


      if(this.number_of_checks_per_swing_ != otherMyClass.number_of_checks_per_swing_) return false;


      if(this.maximum_number_of_adjustment_attempts_ != otherMyClass.maximum_number_of_adjustment_attempts_) return false;


      if(this.maximum_waypoint_adjustment_distance_ != otherMyClass.maximum_waypoint_adjustment_distance_) return false;


      if(this.minimum_adjustment_increment_distance_ != otherMyClass.minimum_adjustment_increment_distance_) return false;


      if(this.maximum_adjustment_increment_distance_ != otherMyClass.maximum_adjustment_increment_distance_) return false;


      if(this.adjustment_increment_distance_gain_ != otherMyClass.adjustment_increment_distance_gain_) return false;


      if(this.minimum_height_above_floor_for_collision_ != otherMyClass.minimum_height_above_floor_for_collision_) return false;


      if(this.swing_height_if_collision_detected_ != otherMyClass.swing_height_if_collision_detected_) return false;


      if(this.minimum_swing_time_ != otherMyClass.minimum_swing_time_) return false;


      if(this.maximum_swing_time_ != otherMyClass.maximum_swing_time_) return false;


      if(this.foot_stub_clearance_ != otherMyClass.foot_stub_clearance_) return false;


      if(this.waypoint_proportion_shift_for_stub_avoidance_ != otherMyClass.waypoint_proportion_shift_for_stub_avoidance_) return false;


      if(this.additional_swing_time_if_expanded_ != otherMyClass.additional_swing_time_if_expanded_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SwingPlannerParametersPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("do_initial_fast_approximation=");
      builder.append(this.do_initial_fast_approximation_);      builder.append(", ");

      builder.append("minimum_swing_foot_clearance=");
      builder.append(this.minimum_swing_foot_clearance_);      builder.append(", ");

      builder.append("fast_approximation_less_clearance=");
      builder.append(this.fast_approximation_less_clearance_);      builder.append(", ");

      builder.append("number_of_checks_per_swing=");
      builder.append(this.number_of_checks_per_swing_);      builder.append(", ");

      builder.append("maximum_number_of_adjustment_attempts=");
      builder.append(this.maximum_number_of_adjustment_attempts_);      builder.append(", ");

      builder.append("maximum_waypoint_adjustment_distance=");
      builder.append(this.maximum_waypoint_adjustment_distance_);      builder.append(", ");

      builder.append("minimum_adjustment_increment_distance=");
      builder.append(this.minimum_adjustment_increment_distance_);      builder.append(", ");

      builder.append("maximum_adjustment_increment_distance=");
      builder.append(this.maximum_adjustment_increment_distance_);      builder.append(", ");

      builder.append("adjustment_increment_distance_gain=");
      builder.append(this.adjustment_increment_distance_gain_);      builder.append(", ");

      builder.append("minimum_height_above_floor_for_collision=");
      builder.append(this.minimum_height_above_floor_for_collision_);      builder.append(", ");

      builder.append("swing_height_if_collision_detected=");
      builder.append(this.swing_height_if_collision_detected_);      builder.append(", ");

      builder.append("minimum_swing_time=");
      builder.append(this.minimum_swing_time_);      builder.append(", ");

      builder.append("maximum_swing_time=");
      builder.append(this.maximum_swing_time_);      builder.append(", ");

      builder.append("foot_stub_clearance=");
      builder.append(this.foot_stub_clearance_);      builder.append(", ");

      builder.append("waypoint_proportion_shift_for_stub_avoidance=");
      builder.append(this.waypoint_proportion_shift_for_stub_avoidance_);      builder.append(", ");

      builder.append("additional_swing_time_if_expanded=");
      builder.append(this.additional_swing_time_if_expanded_);
      builder.append("}");
      return builder.toString();
   }
}
