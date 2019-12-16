package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ValkyrieFootstepPlannerParametersPacket extends Packet<ValkyrieFootstepPlannerParametersPacket> implements Settable<ValkyrieFootstepPlannerParametersPacket>, EpsilonComparable<ValkyrieFootstepPlannerParametersPacket>
{
   /**
          * Default float64 field to represent uninitialized value
          */
   public static final double DEFAULT_NO_VALUE = -11.1;
   /**
            * Returns the ideal step width (i.e. lateral distance) for walking
            */
   public double ideal_footstep_width_ = -11.1;
   /**
            * Returns the farthest backward step length allowed, should be a negative number
            */
   public double minimum_step_length_ = -11.1;
   /**
            * Returns the ideal step length (i.e. forward distance) for walking on flat ground.
            */
   public double ideal_footstep_length_ = -11.1;
   /**
            * Minimum step width allowed.
            * Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
            */
   public double minimum_step_width_ = -11.1;
   /**
            * Maximum step width allowed.
            * Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
            */
   public double maximum_step_width_ = -11.1;
   /**
            * Maximum xy distance from the default squared-up stance as specified by ideal_footstep_width
            */
   public double maximum_step_reach_ = -11.1;
   /**
            * Minimum allowed x distance when y distance is less than min_y_clearance_from_stance
            */
   public double min_x_clearance_from_stance_ = -11.1;
   /**
            * Minimum allowed y distance when x distance is less than min_x_clearance_from_stance
            */
   public double min_y_clearance_from_stance_ = -11.1;
   /**
            * Minimum step yaw, i.e. the farthest angle that steps can point towards each other
            */
   public double minimum_step_yaw_ = -11.1;
   /**
            * Maximum step yaw, i.e. the farthest angle that steps can point away from each other
            */
   public double maximum_step_yaw_ = -11.1;
   /**
            * Reduction factor in yaw depending on step reach. At full reach the yaw constraints are scaled by this factor.
            */
   public double step_yaw_reduction_factor_at_max_reach_ = -11.1;
   /**
            * Maximum step height delta allowed, for both step up and step downs
            */
   public double maximum_step_z_ = -11.1;
   /**
            * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
            */
   public double minimum_foothold_percent_ = -11.1;
   /**
            * Maximum allowed surface incline to place steps
            */
   public double maximum_surface_incline_radians_ = -11.1;
   /**
            * If true, will wiggle each step while planning. If false, only wiggles the final plan.
            * Wiggling while planning slows planner speed by about 50% but is useful when precise foot placements are needed.
            */
   public boolean wiggle_while_planning_;
   /**
            * Amount that the planner will attempt to shift footsteps inside of planar regions.
            * For example, for a value of 0.01 each footstep will be shifted so that its edge is at least 1cm from the planar region's edge
            */
   public double wiggle_inside_delta_ = -11.1;
   /**
            * Maximum xy distance that steps can be shifted to move into planar regions.
            */
   public double maximum_xy_wiggle_distance_ = -11.1;
   /**
            * Maximum yaw distance that steps can be rotated to move into planar regions.
            */
   public double maximum_yaw_wiggle_ = -11.1;
   /**
            * Height threshold to consider an obstacle a "cliff". Steps will not be closer to cliffs than minimum_distance_from_cliff_bottoms
            */
   public double cliff_height_to_avoid_ = -11.1;
   /**
            * Minimum allowed distance to "cliffs"
            */
   public double minimum_distance_from_cliff_bottoms_ = -11.1;
   /**
            * Steps with height changes below this value will be considered step downs. This value should be negative.
            */
   public double flat_ground_lower_threshold_ = -11.1;
   /**
            * Steps with height changes above this value will be considered step ups
            */
   public double flat_ground_upper_threshold_ = -11.1;
   /**
            * Maximum step with if step height change is lower than flat_ground_lower_threshold
            */
   public double maximum_step_width_when_stepping_down_ = -11.1;
   /**
            * Maximum step reach, i.e. xy distance from nominal stance, when stepping down.
            */
   public double maximum_step_reach_when_stepping_down_ = -11.1;
   /**
            * Maximum step with if the step height exceeds flat_ground_upper_threshold
            */
   public double maximum_step_width_when_stepping_up_ = -11.1;
   /**
            * Maximum step reach, i.e. xy distance from nominal stance, when stepping up.
            */
   public double maximum_step_reach_when_stepping_up_ = -11.1;
   /**
            * Scale factor for checking grandparent node (i.e. previous step on same side).
            * If this value is non-zero, if will check the grandparent node on step ups and step downs
            * This value represents how much to scale the corresponding step up/down width/reach restrictions.
            * This value should be in the range 1.0 - 2.0, recommended to be 1.5.
            */
   public double translation_scale_from_grandparent_node_ = -11.1;
   /**
            * Radius around the goal inside which the planner should start to turn to match the goal's orientation
            */
   public double final_turn_proximity_ = -11.1;
   /**
            * Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
            * between consecutive steps and invalidates steps with collisions, see: ObstacleBetweenNodesChecker
            */
   public boolean check_for_path_collisions_;
   /**
            * Sets whether or not the search should check if the body is colliding with the world
            */
   public boolean check_for_body_box_collisions_;
   /**
            * Body box dimensions
            */
   public us.ihmc.euclid.tuple3D.Vector3D body_box_dimensions_;
   /**
            * Body box offset
            */
   public us.ihmc.euclid.tuple3D.Vector3D body_box_offset_;
   /**
            * Sets the number of body collision checks (minimum is 1).
            * Additional checks are done by interpolating between the initial and final steps
            */
   public long number_of_bounding_box_checks_ = 1;
   /**
            * XYZ translation weight vector
            */
   public us.ihmc.euclid.tuple3D.Vector3D translation_weight_;
   /**
            * Yaw-Pitch-Roll weight vector
            */
   public us.ihmc.euclid.tuple3D.Vector3D orientation_weight_;
   /**
            * Contact cost associated with each step. Increase this value if planner is returning extra steps
            */
   public double cost_per_step_ = -11.1;
   /**
            * Weight for partial foothold cost term
            */
   public double foothold_area_weight_ = -11.1;
   /**
            * Heuristic weight for A* search. Higher heuristic weights will result in quicker plan times,
            * but plans might be less optimal.
            */
   public double a_star_heuristics_weight_ = -11.1;

   public ValkyrieFootstepPlannerParametersPacket()
   {
      body_box_dimensions_ = new us.ihmc.euclid.tuple3D.Vector3D();
      body_box_offset_ = new us.ihmc.euclid.tuple3D.Vector3D();
      translation_weight_ = new us.ihmc.euclid.tuple3D.Vector3D();
      orientation_weight_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public ValkyrieFootstepPlannerParametersPacket(ValkyrieFootstepPlannerParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(ValkyrieFootstepPlannerParametersPacket other)
   {
      ideal_footstep_width_ = other.ideal_footstep_width_;

      minimum_step_length_ = other.minimum_step_length_;

      ideal_footstep_length_ = other.ideal_footstep_length_;

      minimum_step_width_ = other.minimum_step_width_;

      maximum_step_width_ = other.maximum_step_width_;

      maximum_step_reach_ = other.maximum_step_reach_;

      min_x_clearance_from_stance_ = other.min_x_clearance_from_stance_;

      min_y_clearance_from_stance_ = other.min_y_clearance_from_stance_;

      minimum_step_yaw_ = other.minimum_step_yaw_;

      maximum_step_yaw_ = other.maximum_step_yaw_;

      step_yaw_reduction_factor_at_max_reach_ = other.step_yaw_reduction_factor_at_max_reach_;

      maximum_step_z_ = other.maximum_step_z_;

      minimum_foothold_percent_ = other.minimum_foothold_percent_;

      maximum_surface_incline_radians_ = other.maximum_surface_incline_radians_;

      wiggle_while_planning_ = other.wiggle_while_planning_;

      wiggle_inside_delta_ = other.wiggle_inside_delta_;

      maximum_xy_wiggle_distance_ = other.maximum_xy_wiggle_distance_;

      maximum_yaw_wiggle_ = other.maximum_yaw_wiggle_;

      cliff_height_to_avoid_ = other.cliff_height_to_avoid_;

      minimum_distance_from_cliff_bottoms_ = other.minimum_distance_from_cliff_bottoms_;

      flat_ground_lower_threshold_ = other.flat_ground_lower_threshold_;

      flat_ground_upper_threshold_ = other.flat_ground_upper_threshold_;

      maximum_step_width_when_stepping_down_ = other.maximum_step_width_when_stepping_down_;

      maximum_step_reach_when_stepping_down_ = other.maximum_step_reach_when_stepping_down_;

      maximum_step_width_when_stepping_up_ = other.maximum_step_width_when_stepping_up_;

      maximum_step_reach_when_stepping_up_ = other.maximum_step_reach_when_stepping_up_;

      translation_scale_from_grandparent_node_ = other.translation_scale_from_grandparent_node_;

      final_turn_proximity_ = other.final_turn_proximity_;

      check_for_path_collisions_ = other.check_for_path_collisions_;

      check_for_body_box_collisions_ = other.check_for_body_box_collisions_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.body_box_dimensions_, body_box_dimensions_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.body_box_offset_, body_box_offset_);
      number_of_bounding_box_checks_ = other.number_of_bounding_box_checks_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.translation_weight_, translation_weight_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.orientation_weight_, orientation_weight_);
      cost_per_step_ = other.cost_per_step_;

      foothold_area_weight_ = other.foothold_area_weight_;

      a_star_heuristics_weight_ = other.a_star_heuristics_weight_;

   }

   /**
            * Returns the ideal step width (i.e. lateral distance) for walking
            */
   public void setIdealFootstepWidth(double ideal_footstep_width)
   {
      ideal_footstep_width_ = ideal_footstep_width;
   }
   /**
            * Returns the ideal step width (i.e. lateral distance) for walking
            */
   public double getIdealFootstepWidth()
   {
      return ideal_footstep_width_;
   }

   /**
            * Returns the farthest backward step length allowed, should be a negative number
            */
   public void setMinimumStepLength(double minimum_step_length)
   {
      minimum_step_length_ = minimum_step_length;
   }
   /**
            * Returns the farthest backward step length allowed, should be a negative number
            */
   public double getMinimumStepLength()
   {
      return minimum_step_length_;
   }

   /**
            * Returns the ideal step length (i.e. forward distance) for walking on flat ground.
            */
   public void setIdealFootstepLength(double ideal_footstep_length)
   {
      ideal_footstep_length_ = ideal_footstep_length;
   }
   /**
            * Returns the ideal step length (i.e. forward distance) for walking on flat ground.
            */
   public double getIdealFootstepLength()
   {
      return ideal_footstep_length_;
   }

   /**
            * Minimum step width allowed.
            * Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
            */
   public void setMinimumStepWidth(double minimum_step_width)
   {
      minimum_step_width_ = minimum_step_width;
   }
   /**
            * Minimum step width allowed.
            * Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
            */
   public double getMinimumStepWidth()
   {
      return minimum_step_width_;
   }

   /**
            * Maximum step width allowed.
            * Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
            */
   public void setMaximumStepWidth(double maximum_step_width)
   {
      maximum_step_width_ = maximum_step_width;
   }
   /**
            * Maximum step width allowed.
            * Step width is the magnitude of the y-position of a footstep expressed in its parent's (i.e. opposite side) frame.
            */
   public double getMaximumStepWidth()
   {
      return maximum_step_width_;
   }

   /**
            * Maximum xy distance from the default squared-up stance as specified by ideal_footstep_width
            */
   public void setMaximumStepReach(double maximum_step_reach)
   {
      maximum_step_reach_ = maximum_step_reach;
   }
   /**
            * Maximum xy distance from the default squared-up stance as specified by ideal_footstep_width
            */
   public double getMaximumStepReach()
   {
      return maximum_step_reach_;
   }

   /**
            * Minimum allowed x distance when y distance is less than min_y_clearance_from_stance
            */
   public void setMinXClearanceFromStance(double min_x_clearance_from_stance)
   {
      min_x_clearance_from_stance_ = min_x_clearance_from_stance;
   }
   /**
            * Minimum allowed x distance when y distance is less than min_y_clearance_from_stance
            */
   public double getMinXClearanceFromStance()
   {
      return min_x_clearance_from_stance_;
   }

   /**
            * Minimum allowed y distance when x distance is less than min_x_clearance_from_stance
            */
   public void setMinYClearanceFromStance(double min_y_clearance_from_stance)
   {
      min_y_clearance_from_stance_ = min_y_clearance_from_stance;
   }
   /**
            * Minimum allowed y distance when x distance is less than min_x_clearance_from_stance
            */
   public double getMinYClearanceFromStance()
   {
      return min_y_clearance_from_stance_;
   }

   /**
            * Minimum step yaw, i.e. the farthest angle that steps can point towards each other
            */
   public void setMinimumStepYaw(double minimum_step_yaw)
   {
      minimum_step_yaw_ = minimum_step_yaw;
   }
   /**
            * Minimum step yaw, i.e. the farthest angle that steps can point towards each other
            */
   public double getMinimumStepYaw()
   {
      return minimum_step_yaw_;
   }

   /**
            * Maximum step yaw, i.e. the farthest angle that steps can point away from each other
            */
   public void setMaximumStepYaw(double maximum_step_yaw)
   {
      maximum_step_yaw_ = maximum_step_yaw;
   }
   /**
            * Maximum step yaw, i.e. the farthest angle that steps can point away from each other
            */
   public double getMaximumStepYaw()
   {
      return maximum_step_yaw_;
   }

   /**
            * Reduction factor in yaw depending on step reach. At full reach the yaw constraints are scaled by this factor.
            */
   public void setStepYawReductionFactorAtMaxReach(double step_yaw_reduction_factor_at_max_reach)
   {
      step_yaw_reduction_factor_at_max_reach_ = step_yaw_reduction_factor_at_max_reach;
   }
   /**
            * Reduction factor in yaw depending on step reach. At full reach the yaw constraints are scaled by this factor.
            */
   public double getStepYawReductionFactorAtMaxReach()
   {
      return step_yaw_reduction_factor_at_max_reach_;
   }

   /**
            * Maximum step height delta allowed, for both step up and step downs
            */
   public void setMaximumStepZ(double maximum_step_z)
   {
      maximum_step_z_ = maximum_step_z;
   }
   /**
            * Maximum step height delta allowed, for both step up and step downs
            */
   public double getMaximumStepZ()
   {
      return maximum_step_z_;
   }

   /**
            * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
            */
   public void setMinimumFootholdPercent(double minimum_foothold_percent)
   {
      minimum_foothold_percent_ = minimum_foothold_percent;
   }
   /**
            * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
            */
   public double getMinimumFootholdPercent()
   {
      return minimum_foothold_percent_;
   }

   /**
            * Maximum allowed surface incline to place steps
            */
   public void setMaximumSurfaceInclineRadians(double maximum_surface_incline_radians)
   {
      maximum_surface_incline_radians_ = maximum_surface_incline_radians;
   }
   /**
            * Maximum allowed surface incline to place steps
            */
   public double getMaximumSurfaceInclineRadians()
   {
      return maximum_surface_incline_radians_;
   }

   /**
            * If true, will wiggle each step while planning. If false, only wiggles the final plan.
            * Wiggling while planning slows planner speed by about 50% but is useful when precise foot placements are needed.
            */
   public void setWiggleWhilePlanning(boolean wiggle_while_planning)
   {
      wiggle_while_planning_ = wiggle_while_planning;
   }
   /**
            * If true, will wiggle each step while planning. If false, only wiggles the final plan.
            * Wiggling while planning slows planner speed by about 50% but is useful when precise foot placements are needed.
            */
   public boolean getWiggleWhilePlanning()
   {
      return wiggle_while_planning_;
   }

   /**
            * Amount that the planner will attempt to shift footsteps inside of planar regions.
            * For example, for a value of 0.01 each footstep will be shifted so that its edge is at least 1cm from the planar region's edge
            */
   public void setWiggleInsideDelta(double wiggle_inside_delta)
   {
      wiggle_inside_delta_ = wiggle_inside_delta;
   }
   /**
            * Amount that the planner will attempt to shift footsteps inside of planar regions.
            * For example, for a value of 0.01 each footstep will be shifted so that its edge is at least 1cm from the planar region's edge
            */
   public double getWiggleInsideDelta()
   {
      return wiggle_inside_delta_;
   }

   /**
            * Maximum xy distance that steps can be shifted to move into planar regions.
            */
   public void setMaximumXyWiggleDistance(double maximum_xy_wiggle_distance)
   {
      maximum_xy_wiggle_distance_ = maximum_xy_wiggle_distance;
   }
   /**
            * Maximum xy distance that steps can be shifted to move into planar regions.
            */
   public double getMaximumXyWiggleDistance()
   {
      return maximum_xy_wiggle_distance_;
   }

   /**
            * Maximum yaw distance that steps can be rotated to move into planar regions.
            */
   public void setMaximumYawWiggle(double maximum_yaw_wiggle)
   {
      maximum_yaw_wiggle_ = maximum_yaw_wiggle;
   }
   /**
            * Maximum yaw distance that steps can be rotated to move into planar regions.
            */
   public double getMaximumYawWiggle()
   {
      return maximum_yaw_wiggle_;
   }

   /**
            * Height threshold to consider an obstacle a "cliff". Steps will not be closer to cliffs than minimum_distance_from_cliff_bottoms
            */
   public void setCliffHeightToAvoid(double cliff_height_to_avoid)
   {
      cliff_height_to_avoid_ = cliff_height_to_avoid;
   }
   /**
            * Height threshold to consider an obstacle a "cliff". Steps will not be closer to cliffs than minimum_distance_from_cliff_bottoms
            */
   public double getCliffHeightToAvoid()
   {
      return cliff_height_to_avoid_;
   }

   /**
            * Minimum allowed distance to "cliffs"
            */
   public void setMinimumDistanceFromCliffBottoms(double minimum_distance_from_cliff_bottoms)
   {
      minimum_distance_from_cliff_bottoms_ = minimum_distance_from_cliff_bottoms;
   }
   /**
            * Minimum allowed distance to "cliffs"
            */
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimum_distance_from_cliff_bottoms_;
   }

   /**
            * Steps with height changes below this value will be considered step downs. This value should be negative.
            */
   public void setFlatGroundLowerThreshold(double flat_ground_lower_threshold)
   {
      flat_ground_lower_threshold_ = flat_ground_lower_threshold;
   }
   /**
            * Steps with height changes below this value will be considered step downs. This value should be negative.
            */
   public double getFlatGroundLowerThreshold()
   {
      return flat_ground_lower_threshold_;
   }

   /**
            * Steps with height changes above this value will be considered step ups
            */
   public void setFlatGroundUpperThreshold(double flat_ground_upper_threshold)
   {
      flat_ground_upper_threshold_ = flat_ground_upper_threshold;
   }
   /**
            * Steps with height changes above this value will be considered step ups
            */
   public double getFlatGroundUpperThreshold()
   {
      return flat_ground_upper_threshold_;
   }

   /**
            * Maximum step with if step height change is lower than flat_ground_lower_threshold
            */
   public void setMaximumStepWidthWhenSteppingDown(double maximum_step_width_when_stepping_down)
   {
      maximum_step_width_when_stepping_down_ = maximum_step_width_when_stepping_down;
   }
   /**
            * Maximum step with if step height change is lower than flat_ground_lower_threshold
            */
   public double getMaximumStepWidthWhenSteppingDown()
   {
      return maximum_step_width_when_stepping_down_;
   }

   /**
            * Maximum step reach, i.e. xy distance from nominal stance, when stepping down.
            */
   public void setMaximumStepReachWhenSteppingDown(double maximum_step_reach_when_stepping_down)
   {
      maximum_step_reach_when_stepping_down_ = maximum_step_reach_when_stepping_down;
   }
   /**
            * Maximum step reach, i.e. xy distance from nominal stance, when stepping down.
            */
   public double getMaximumStepReachWhenSteppingDown()
   {
      return maximum_step_reach_when_stepping_down_;
   }

   /**
            * Maximum step with if the step height exceeds flat_ground_upper_threshold
            */
   public void setMaximumStepWidthWhenSteppingUp(double maximum_step_width_when_stepping_up)
   {
      maximum_step_width_when_stepping_up_ = maximum_step_width_when_stepping_up;
   }
   /**
            * Maximum step with if the step height exceeds flat_ground_upper_threshold
            */
   public double getMaximumStepWidthWhenSteppingUp()
   {
      return maximum_step_width_when_stepping_up_;
   }

   /**
            * Maximum step reach, i.e. xy distance from nominal stance, when stepping up.
            */
   public void setMaximumStepReachWhenSteppingUp(double maximum_step_reach_when_stepping_up)
   {
      maximum_step_reach_when_stepping_up_ = maximum_step_reach_when_stepping_up;
   }
   /**
            * Maximum step reach, i.e. xy distance from nominal stance, when stepping up.
            */
   public double getMaximumStepReachWhenSteppingUp()
   {
      return maximum_step_reach_when_stepping_up_;
   }

   /**
            * Scale factor for checking grandparent node (i.e. previous step on same side).
            * If this value is non-zero, if will check the grandparent node on step ups and step downs
            * This value represents how much to scale the corresponding step up/down width/reach restrictions.
            * This value should be in the range 1.0 - 2.0, recommended to be 1.5.
            */
   public void setTranslationScaleFromGrandparentNode(double translation_scale_from_grandparent_node)
   {
      translation_scale_from_grandparent_node_ = translation_scale_from_grandparent_node;
   }
   /**
            * Scale factor for checking grandparent node (i.e. previous step on same side).
            * If this value is non-zero, if will check the grandparent node on step ups and step downs
            * This value represents how much to scale the corresponding step up/down width/reach restrictions.
            * This value should be in the range 1.0 - 2.0, recommended to be 1.5.
            */
   public double getTranslationScaleFromGrandparentNode()
   {
      return translation_scale_from_grandparent_node_;
   }

   /**
            * Radius around the goal inside which the planner should start to turn to match the goal's orientation
            */
   public void setFinalTurnProximity(double final_turn_proximity)
   {
      final_turn_proximity_ = final_turn_proximity;
   }
   /**
            * Radius around the goal inside which the planner should start to turn to match the goal's orientation
            */
   public double getFinalTurnProximity()
   {
      return final_turn_proximity_;
   }

   /**
            * Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
            * between consecutive steps and invalidates steps with collisions, see: ObstacleBetweenNodesChecker
            */
   public void setCheckForPathCollisions(boolean check_for_path_collisions)
   {
      check_for_path_collisions_ = check_for_path_collisions;
   }
   /**
            * Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
            * between consecutive steps and invalidates steps with collisions, see: ObstacleBetweenNodesChecker
            */
   public boolean getCheckForPathCollisions()
   {
      return check_for_path_collisions_;
   }

   /**
            * Sets whether or not the search should check if the body is colliding with the world
            */
   public void setCheckForBodyBoxCollisions(boolean check_for_body_box_collisions)
   {
      check_for_body_box_collisions_ = check_for_body_box_collisions;
   }
   /**
            * Sets whether or not the search should check if the body is colliding with the world
            */
   public boolean getCheckForBodyBoxCollisions()
   {
      return check_for_body_box_collisions_;
   }


   /**
            * Body box dimensions
            */
   public us.ihmc.euclid.tuple3D.Vector3D getBodyBoxDimensions()
   {
      return body_box_dimensions_;
   }


   /**
            * Body box offset
            */
   public us.ihmc.euclid.tuple3D.Vector3D getBodyBoxOffset()
   {
      return body_box_offset_;
   }

   /**
            * Sets the number of body collision checks (minimum is 1).
            * Additional checks are done by interpolating between the initial and final steps
            */
   public void setNumberOfBoundingBoxChecks(long number_of_bounding_box_checks)
   {
      number_of_bounding_box_checks_ = number_of_bounding_box_checks;
   }
   /**
            * Sets the number of body collision checks (minimum is 1).
            * Additional checks are done by interpolating between the initial and final steps
            */
   public long getNumberOfBoundingBoxChecks()
   {
      return number_of_bounding_box_checks_;
   }


   /**
            * XYZ translation weight vector
            */
   public us.ihmc.euclid.tuple3D.Vector3D getTranslationWeight()
   {
      return translation_weight_;
   }


   /**
            * Yaw-Pitch-Roll weight vector
            */
   public us.ihmc.euclid.tuple3D.Vector3D getOrientationWeight()
   {
      return orientation_weight_;
   }

   /**
            * Contact cost associated with each step. Increase this value if planner is returning extra steps
            */
   public void setCostPerStep(double cost_per_step)
   {
      cost_per_step_ = cost_per_step;
   }
   /**
            * Contact cost associated with each step. Increase this value if planner is returning extra steps
            */
   public double getCostPerStep()
   {
      return cost_per_step_;
   }

   /**
            * Weight for partial foothold cost term
            */
   public void setFootholdAreaWeight(double foothold_area_weight)
   {
      foothold_area_weight_ = foothold_area_weight;
   }
   /**
            * Weight for partial foothold cost term
            */
   public double getFootholdAreaWeight()
   {
      return foothold_area_weight_;
   }

   /**
            * Heuristic weight for A* search. Higher heuristic weights will result in quicker plan times,
            * but plans might be less optimal.
            */
   public void setAStarHeuristicsWeight(double a_star_heuristics_weight)
   {
      a_star_heuristics_weight_ = a_star_heuristics_weight;
   }
   /**
            * Heuristic weight for A* search. Higher heuristic weights will result in quicker plan times,
            * but plans might be less optimal.
            */
   public double getAStarHeuristicsWeight()
   {
      return a_star_heuristics_weight_;
   }


   public static Supplier<ValkyrieFootstepPlannerParametersPacketPubSubType> getPubSubType()
   {
      return ValkyrieFootstepPlannerParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ValkyrieFootstepPlannerParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ValkyrieFootstepPlannerParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_footstep_width_, other.ideal_footstep_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_length_, other.minimum_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_footstep_length_, other.ideal_footstep_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_width_, other.minimum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_, other.maximum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_, other.maximum_step_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_x_clearance_from_stance_, other.min_x_clearance_from_stance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_y_clearance_from_stance_, other.min_y_clearance_from_stance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_yaw_, other.minimum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_yaw_, other.maximum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_yaw_reduction_factor_at_max_reach_, other.step_yaw_reduction_factor_at_max_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_, other.maximum_step_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_foothold_percent_, other.minimum_foothold_percent_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_surface_incline_radians_, other.maximum_surface_incline_radians_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.wiggle_while_planning_, other.wiggle_while_planning_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wiggle_inside_delta_, other.wiggle_inside_delta_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_xy_wiggle_distance_, other.maximum_xy_wiggle_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_yaw_wiggle_, other.maximum_yaw_wiggle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cliff_height_to_avoid_, other.cliff_height_to_avoid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_distance_from_cliff_bottoms_, other.minimum_distance_from_cliff_bottoms_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flat_ground_lower_threshold_, other.flat_ground_lower_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flat_ground_upper_threshold_, other.flat_ground_upper_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_when_stepping_down_, other.maximum_step_width_when_stepping_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_when_stepping_down_, other.maximum_step_reach_when_stepping_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_when_stepping_up_, other.maximum_step_width_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_when_stepping_up_, other.maximum_step_reach_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.translation_scale_from_grandparent_node_, other.translation_scale_from_grandparent_node_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_turn_proximity_, other.final_turn_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_for_path_collisions_, other.check_for_path_collisions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_for_body_box_collisions_, other.check_for_body_box_collisions_, epsilon)) return false;

      if (!this.body_box_dimensions_.epsilonEquals(other.body_box_dimensions_, epsilon)) return false;
      if (!this.body_box_offset_.epsilonEquals(other.body_box_offset_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_bounding_box_checks_, other.number_of_bounding_box_checks_, epsilon)) return false;

      if (!this.translation_weight_.epsilonEquals(other.translation_weight_, epsilon)) return false;
      if (!this.orientation_weight_.epsilonEquals(other.orientation_weight_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cost_per_step_, other.cost_per_step_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foothold_area_weight_, other.foothold_area_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.a_star_heuristics_weight_, other.a_star_heuristics_weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ValkyrieFootstepPlannerParametersPacket)) return false;

      ValkyrieFootstepPlannerParametersPacket otherMyClass = (ValkyrieFootstepPlannerParametersPacket) other;

      if(this.ideal_footstep_width_ != otherMyClass.ideal_footstep_width_) return false;

      if(this.minimum_step_length_ != otherMyClass.minimum_step_length_) return false;

      if(this.ideal_footstep_length_ != otherMyClass.ideal_footstep_length_) return false;

      if(this.minimum_step_width_ != otherMyClass.minimum_step_width_) return false;

      if(this.maximum_step_width_ != otherMyClass.maximum_step_width_) return false;

      if(this.maximum_step_reach_ != otherMyClass.maximum_step_reach_) return false;

      if(this.min_x_clearance_from_stance_ != otherMyClass.min_x_clearance_from_stance_) return false;

      if(this.min_y_clearance_from_stance_ != otherMyClass.min_y_clearance_from_stance_) return false;

      if(this.minimum_step_yaw_ != otherMyClass.minimum_step_yaw_) return false;

      if(this.maximum_step_yaw_ != otherMyClass.maximum_step_yaw_) return false;

      if(this.step_yaw_reduction_factor_at_max_reach_ != otherMyClass.step_yaw_reduction_factor_at_max_reach_) return false;

      if(this.maximum_step_z_ != otherMyClass.maximum_step_z_) return false;

      if(this.minimum_foothold_percent_ != otherMyClass.minimum_foothold_percent_) return false;

      if(this.maximum_surface_incline_radians_ != otherMyClass.maximum_surface_incline_radians_) return false;

      if(this.wiggle_while_planning_ != otherMyClass.wiggle_while_planning_) return false;

      if(this.wiggle_inside_delta_ != otherMyClass.wiggle_inside_delta_) return false;

      if(this.maximum_xy_wiggle_distance_ != otherMyClass.maximum_xy_wiggle_distance_) return false;

      if(this.maximum_yaw_wiggle_ != otherMyClass.maximum_yaw_wiggle_) return false;

      if(this.cliff_height_to_avoid_ != otherMyClass.cliff_height_to_avoid_) return false;

      if(this.minimum_distance_from_cliff_bottoms_ != otherMyClass.minimum_distance_from_cliff_bottoms_) return false;

      if(this.flat_ground_lower_threshold_ != otherMyClass.flat_ground_lower_threshold_) return false;

      if(this.flat_ground_upper_threshold_ != otherMyClass.flat_ground_upper_threshold_) return false;

      if(this.maximum_step_width_when_stepping_down_ != otherMyClass.maximum_step_width_when_stepping_down_) return false;

      if(this.maximum_step_reach_when_stepping_down_ != otherMyClass.maximum_step_reach_when_stepping_down_) return false;

      if(this.maximum_step_width_when_stepping_up_ != otherMyClass.maximum_step_width_when_stepping_up_) return false;

      if(this.maximum_step_reach_when_stepping_up_ != otherMyClass.maximum_step_reach_when_stepping_up_) return false;

      if(this.translation_scale_from_grandparent_node_ != otherMyClass.translation_scale_from_grandparent_node_) return false;

      if(this.final_turn_proximity_ != otherMyClass.final_turn_proximity_) return false;

      if(this.check_for_path_collisions_ != otherMyClass.check_for_path_collisions_) return false;

      if(this.check_for_body_box_collisions_ != otherMyClass.check_for_body_box_collisions_) return false;

      if (!this.body_box_dimensions_.equals(otherMyClass.body_box_dimensions_)) return false;
      if (!this.body_box_offset_.equals(otherMyClass.body_box_offset_)) return false;
      if(this.number_of_bounding_box_checks_ != otherMyClass.number_of_bounding_box_checks_) return false;

      if (!this.translation_weight_.equals(otherMyClass.translation_weight_)) return false;
      if (!this.orientation_weight_.equals(otherMyClass.orientation_weight_)) return false;
      if(this.cost_per_step_ != otherMyClass.cost_per_step_) return false;

      if(this.foothold_area_weight_ != otherMyClass.foothold_area_weight_) return false;

      if(this.a_star_heuristics_weight_ != otherMyClass.a_star_heuristics_weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValkyrieFootstepPlannerParametersPacket {");
      builder.append("ideal_footstep_width=");
      builder.append(this.ideal_footstep_width_);      builder.append(", ");
      builder.append("minimum_step_length=");
      builder.append(this.minimum_step_length_);      builder.append(", ");
      builder.append("ideal_footstep_length=");
      builder.append(this.ideal_footstep_length_);      builder.append(", ");
      builder.append("minimum_step_width=");
      builder.append(this.minimum_step_width_);      builder.append(", ");
      builder.append("maximum_step_width=");
      builder.append(this.maximum_step_width_);      builder.append(", ");
      builder.append("maximum_step_reach=");
      builder.append(this.maximum_step_reach_);      builder.append(", ");
      builder.append("min_x_clearance_from_stance=");
      builder.append(this.min_x_clearance_from_stance_);      builder.append(", ");
      builder.append("min_y_clearance_from_stance=");
      builder.append(this.min_y_clearance_from_stance_);      builder.append(", ");
      builder.append("minimum_step_yaw=");
      builder.append(this.minimum_step_yaw_);      builder.append(", ");
      builder.append("maximum_step_yaw=");
      builder.append(this.maximum_step_yaw_);      builder.append(", ");
      builder.append("step_yaw_reduction_factor_at_max_reach=");
      builder.append(this.step_yaw_reduction_factor_at_max_reach_);      builder.append(", ");
      builder.append("maximum_step_z=");
      builder.append(this.maximum_step_z_);      builder.append(", ");
      builder.append("minimum_foothold_percent=");
      builder.append(this.minimum_foothold_percent_);      builder.append(", ");
      builder.append("maximum_surface_incline_radians=");
      builder.append(this.maximum_surface_incline_radians_);      builder.append(", ");
      builder.append("wiggle_while_planning=");
      builder.append(this.wiggle_while_planning_);      builder.append(", ");
      builder.append("wiggle_inside_delta=");
      builder.append(this.wiggle_inside_delta_);      builder.append(", ");
      builder.append("maximum_xy_wiggle_distance=");
      builder.append(this.maximum_xy_wiggle_distance_);      builder.append(", ");
      builder.append("maximum_yaw_wiggle=");
      builder.append(this.maximum_yaw_wiggle_);      builder.append(", ");
      builder.append("cliff_height_to_avoid=");
      builder.append(this.cliff_height_to_avoid_);      builder.append(", ");
      builder.append("minimum_distance_from_cliff_bottoms=");
      builder.append(this.minimum_distance_from_cliff_bottoms_);      builder.append(", ");
      builder.append("flat_ground_lower_threshold=");
      builder.append(this.flat_ground_lower_threshold_);      builder.append(", ");
      builder.append("flat_ground_upper_threshold=");
      builder.append(this.flat_ground_upper_threshold_);      builder.append(", ");
      builder.append("maximum_step_width_when_stepping_down=");
      builder.append(this.maximum_step_width_when_stepping_down_);      builder.append(", ");
      builder.append("maximum_step_reach_when_stepping_down=");
      builder.append(this.maximum_step_reach_when_stepping_down_);      builder.append(", ");
      builder.append("maximum_step_width_when_stepping_up=");
      builder.append(this.maximum_step_width_when_stepping_up_);      builder.append(", ");
      builder.append("maximum_step_reach_when_stepping_up=");
      builder.append(this.maximum_step_reach_when_stepping_up_);      builder.append(", ");
      builder.append("translation_scale_from_grandparent_node=");
      builder.append(this.translation_scale_from_grandparent_node_);      builder.append(", ");
      builder.append("final_turn_proximity=");
      builder.append(this.final_turn_proximity_);      builder.append(", ");
      builder.append("check_for_path_collisions=");
      builder.append(this.check_for_path_collisions_);      builder.append(", ");
      builder.append("check_for_body_box_collisions=");
      builder.append(this.check_for_body_box_collisions_);      builder.append(", ");
      builder.append("body_box_dimensions=");
      builder.append(this.body_box_dimensions_);      builder.append(", ");
      builder.append("body_box_offset=");
      builder.append(this.body_box_offset_);      builder.append(", ");
      builder.append("number_of_bounding_box_checks=");
      builder.append(this.number_of_bounding_box_checks_);      builder.append(", ");
      builder.append("translation_weight=");
      builder.append(this.translation_weight_);      builder.append(", ");
      builder.append("orientation_weight=");
      builder.append(this.orientation_weight_);      builder.append(", ");
      builder.append("cost_per_step=");
      builder.append(this.cost_per_step_);      builder.append(", ");
      builder.append("foothold_area_weight=");
      builder.append(this.foothold_area_weight_);      builder.append(", ");
      builder.append("a_star_heuristics_weight=");
      builder.append(this.a_star_heuristics_weight_);
      builder.append("}");
      return builder.toString();
   }
}
