package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       * Maximum step reach when stepping up.
       * 
       * Long steps forward are rejected by the planner if one of two criteria are met:
       * The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * - OR -
       * The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * - AND -
       * The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * Maximum step width when stepping up.
       * 
       * Long steps forward are rejected by the planner if one of two criteria are met:
       * The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * - OR -
       * The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * - AND -
       * The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * Step height for considering stepping up.
       * 
       * Long steps forward are rejected by the planner if one of two criteria are met:
       * The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * - OR -
       * The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * - AND -
       * The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       */
public class FootstepPlannerParametersPacket extends Packet<FootstepPlannerParametersPacket> implements Settable<FootstepPlannerParametersPacket>, EpsilonComparable<FootstepPlannerParametersPacket>
{
   public static final double DEFAULT_NO_VALUE = -11.1;
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
            * to run slower.
            */
   public boolean check_for_body_box_collisions_;
   /**
            * Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
            * between consecutive steps and invalidates steps with collisions, see: ObstacleBetweenNodesChecker
            */
   public boolean check_for_path_collisions_;
   /**
            * Returns the ideal step width for walking on flat ground.
            */
   public double ideal_footstep_width_ = -11.1;
   /**
            * Returns the ideal step length for walking on flat ground.
            */
   public double ideal_footstep_length_ = -11.1;
   /**
            * Returns the ideal step side step width when "shuffling" sideways.
            */
   public double ideal_side_step_width_ = -11.1;
   /**
            * Returns the ideal length when walking backwards. This value is positive.
            */
   public double ideal_back_step_length_ = -11.1;
   /**
            * Returns ideal step length when the vertical height between the start-of-swing and stance feet are at maximum allowed height.
            */
   public double ideal_step_length_at_max_step_z_ = -11.1;
   /**
            * The planner will try to shift footsteps inside of a region so that this value is the minimum distance from the step
            * to the edge. A negative value means the footstep can overhang a region.
            */
   public double wiggle_inside_delta_target_ = -11.1;
   /**
            * This parameter only is used if wiggle_while_planning is true. If a step cannot be wiggled inside by this amount or more,
            * it will be rejected. Note that if {wiggle_while_planning if false, it's always best effort on the final plan.
            */
   public double wiggle_inside_delta_minimum_ = -11.1;
   /**
            * Maximum xy-distance the planner will consider for candidate steps.
            * Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
            */
   public double maximum_step_reach_ = -11.1;
   /**
            * Maximum yaw between consecutive footsteps
            * 
            * A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
            * 
            * This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
            * space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
            * its maximum reach.
            */
   public double maximum_step_yaw_ = -11.1;
   /**
            * Minimum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public double minimum_step_width_ = -11.1;
   /**
            * Minimum step length the planner will consider for candidate steps.
            * 
            * Step length refers to the x-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public double minimum_step_length_ = -11.1;
   /**
            * Minimum step yaw.
            */
   public double minimum_step_yaw_ = -11.1;
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double maximum_step_reach_when_stepping_up_ = -11.1;
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double maximum_step_width_when_stepping_up_ = -11.1;
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double maximum_step_z_when_stepping_up_ = -11.1;
   /**
            * Maximum step length when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double maximum_step_x_when_forward_and_down_ = -11.1;
   /**
            * Maximum step width when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double maximum_step_y_when_forward_and_down_ = -11.1;
   /**
            * Maximum step height when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double maximum_step_z_when_forward_and_down_ = -11.1;
   /**
            * Maximum vertical distance between consecutive footsteps
            * 
            * A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
            * z-up sole frame.
            */
   public double maximum_step_z_ = -11.1;
   /**
            * Maximum vertical distance between start-of-swing and touchdown
            */
   public double maximum_swing_z_ = -11.1;
   /**
            * Maximum xy distance between start-of-swing and touchdown
            */
   public double maximum_swing_reach_ = -11.1;
   /**
            * Maximum vertical distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
            * 
            * The maximum depth is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
            * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
            */
   public double minimum_step_z_when_fully_pitched_ = -11.1;
   /**
            * Maximum forward distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
            * 
            * The maximum distance is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
            * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
            */
   public double maximum_step_x_when_fully_pitched_ = -11.1;
   /**
            * This is the reduction factor for the max yaw when the step is at max reach.
            * This means that, when the footstep is at its maximum distance, this is the fraction reduction of the max yaw.
            * If this returns 0.0, the max yaw is not modified, even at full reach.
            * If this returns 1.0, the max yaw is 0 at full reach.
            * 
            * That is,
            * modifiedMaxYaw = (1.0 - reach / maxReach) * maxYaw + reach / maxReach * (1.0 - alpha) * maxYaw
            */
   public double step_yaw_reduction_factor_at_max_reach_ = -11.1;
   /**
            * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
            * 
            * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
            */
   public double minimum_foothold_percent_ = 0.9;
   /**
            * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
            * then the value specified here.
            * 
            * More specifically, if a footstep has an associated planar region and that regions surface normal has a
            * z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
            */
   public double minimum_surface_incline_radians_ = -11.1;
   /**
            * The wiggler can either run as a post-processor on a resulting plan or on each candidate step while planning.
            * If true, this wiggles each candidate step, which will slow down plan times but resulting plans will be guarunteed to match step constraints.
            */
   public boolean wiggle_while_planning_;
   /**
            * If wiggle_while_planning is true, this will reject a step if the wiggle meet the specified parameters. If it's false the wiggle does a best effort
            */
   public boolean reject_if_wiggle_not_satisfied_;
   /**
            * There are two solvers for wiggling the step, one constrains to the region's convex hull and the other to the region's concave hull,
            * this toggles between them.
            */
   public boolean enable_concave_hull_wiggler_;
   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
            * distance the planner will use
            */
   public double maximum_xy_wiggle_distance_ = -11.1;
   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum yaw
            * distance the planner will use
            */
   public double maximum_yaw_wiggle_ = -11.1;
   /**
            * When snapping a candidate footstep to a planar region, its possible that another planar region
            * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
            * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
            * otherwise it will.
            */
   public double maximum_z_penetration_on_valley_regions_ = -11.1;
   /**
            * Maximum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
            */
   public double maximum_step_width_ = -11.1;
   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double cliff_base_height_to_avoid_ = -11.1;
   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffBaseHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double minimum_distance_from_cliff_bottoms_ = -11.1;
   /**
            * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffTops away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double cliff_top_height_to_avoid_ = -11.1;
   /**
            * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffTopHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double minimum_distance_from_cliff_tops_ = -11.1;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box height.
            */
   public double body_box_height_ = -11.1;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box depth.
            */
   public double body_box_depth_ = -11.1;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box width.
            */
   public double body_box_width_ = -11.1;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double body_box_base_x_ = -11.1;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double body_box_base_y_ = -11.1;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double body_box_base_z_ = -11.1;
   /**
            * Maximum height above a stance step that a candidate step is snapped to. Regions above this height are ignored.
            * Intended to avoid ceilings or obstacles that are above the top of the robot
            */
   public double maximum_snap_height_ = -11.1;
   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public double min_clearance_from_stance_ = -11.1;
   /**
            * Radius around the goal inside which the planner should start to turn to match the goal's orientation
            */
   public double final_turn_proximity_ = -11.1;
   /**
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public double yaw_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public double pitch_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public double roll_weight_ = -11.1;
   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public double forward_weight_ = -11.1;
   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public double lateral_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public double step_up_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public double step_down_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public double long_step_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public double foothold_area_weight_ = -11.1;
   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public double cost_per_step_ = -11.1;
   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public double a_star_heuristics_weight_ = -11.1;
   /**
            * This sets how many bounding box checks to perform. If this value is 1, only the final footstep is checked.
            * Additional checks are done by interpolating between the start and end steps
            */
   public long number_of_bounding_box_checks_ = 1;
   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public double maximum_2d_distance_from_bounding_box_to_penalize_ = -11.1;
   /**
            * If the robot's mid-foot pose is within this distance of the body path, it will match the body path heading.
            * Otherwise, it will turn towards the body path
            */
   public double distance_from_path_tolerance_ = -11.1;
   /**
            * If the robot's mid-foot pose oriented within this threshold of the body path's heading, it will match the body path heading.
            * Otherwise, it will turn in plance towards the body path
            */
   public double delta_yaw_from_reference_tolerance_ = -11.1;
   /**
            * Maximum steps considered at each iteration. If more than this number of steps are available, the closest steps to the
            * ideal step are considered and the others are ignored. Set to non-positive number to disable
            */
   public int maximum_branch_factor_ = -1;
   /**
            * If true, enables a mask that reduces the number of calculated steps away from the ideal step
            */
   public boolean enable_expansion_mask_ = true;
   /**
            * If true will try to wiggle steps away from shin collisions. Collisions are checked against all regions.
            * Enable concave hull wiggler must be true in order for the shin collision checker to run.
            */
   public boolean enable_shin_collision_check_;
   /**
            * How far the shin collision cylinder extends from the toe
            */
   public double shin_toe_clearance_ = -11.1;
   /**
            * How far the shin collision cylinder extends from the heel
            */
   public double shin_heel_clearance_ = -11.1;
   /**
            * Length of the shin collidable cylinder
            */
   public double shin_length_ = -11.1;
   /**
            * Height offset of shin collidable cylinder
            */
   public double shin_height_offet_ = -11.1;

   public FootstepPlannerParametersPacket()
   {
   }

   public FootstepPlannerParametersPacket(FootstepPlannerParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      check_for_body_box_collisions_ = other.check_for_body_box_collisions_;

      check_for_path_collisions_ = other.check_for_path_collisions_;

      ideal_footstep_width_ = other.ideal_footstep_width_;

      ideal_footstep_length_ = other.ideal_footstep_length_;

      ideal_side_step_width_ = other.ideal_side_step_width_;

      ideal_back_step_length_ = other.ideal_back_step_length_;

      ideal_step_length_at_max_step_z_ = other.ideal_step_length_at_max_step_z_;

      wiggle_inside_delta_target_ = other.wiggle_inside_delta_target_;

      wiggle_inside_delta_minimum_ = other.wiggle_inside_delta_minimum_;

      maximum_step_reach_ = other.maximum_step_reach_;

      maximum_step_yaw_ = other.maximum_step_yaw_;

      minimum_step_width_ = other.minimum_step_width_;

      minimum_step_length_ = other.minimum_step_length_;

      minimum_step_yaw_ = other.minimum_step_yaw_;

      maximum_step_reach_when_stepping_up_ = other.maximum_step_reach_when_stepping_up_;

      maximum_step_width_when_stepping_up_ = other.maximum_step_width_when_stepping_up_;

      maximum_step_z_when_stepping_up_ = other.maximum_step_z_when_stepping_up_;

      maximum_step_x_when_forward_and_down_ = other.maximum_step_x_when_forward_and_down_;

      maximum_step_y_when_forward_and_down_ = other.maximum_step_y_when_forward_and_down_;

      maximum_step_z_when_forward_and_down_ = other.maximum_step_z_when_forward_and_down_;

      maximum_step_z_ = other.maximum_step_z_;

      maximum_swing_z_ = other.maximum_swing_z_;

      maximum_swing_reach_ = other.maximum_swing_reach_;

      minimum_step_z_when_fully_pitched_ = other.minimum_step_z_when_fully_pitched_;

      maximum_step_x_when_fully_pitched_ = other.maximum_step_x_when_fully_pitched_;

      step_yaw_reduction_factor_at_max_reach_ = other.step_yaw_reduction_factor_at_max_reach_;

      minimum_foothold_percent_ = other.minimum_foothold_percent_;

      minimum_surface_incline_radians_ = other.minimum_surface_incline_radians_;

      wiggle_while_planning_ = other.wiggle_while_planning_;

      reject_if_wiggle_not_satisfied_ = other.reject_if_wiggle_not_satisfied_;

      enable_concave_hull_wiggler_ = other.enable_concave_hull_wiggler_;

      maximum_xy_wiggle_distance_ = other.maximum_xy_wiggle_distance_;

      maximum_yaw_wiggle_ = other.maximum_yaw_wiggle_;

      maximum_z_penetration_on_valley_regions_ = other.maximum_z_penetration_on_valley_regions_;

      maximum_step_width_ = other.maximum_step_width_;

      cliff_base_height_to_avoid_ = other.cliff_base_height_to_avoid_;

      minimum_distance_from_cliff_bottoms_ = other.minimum_distance_from_cliff_bottoms_;

      cliff_top_height_to_avoid_ = other.cliff_top_height_to_avoid_;

      minimum_distance_from_cliff_tops_ = other.minimum_distance_from_cliff_tops_;

      body_box_height_ = other.body_box_height_;

      body_box_depth_ = other.body_box_depth_;

      body_box_width_ = other.body_box_width_;

      body_box_base_x_ = other.body_box_base_x_;

      body_box_base_y_ = other.body_box_base_y_;

      body_box_base_z_ = other.body_box_base_z_;

      maximum_snap_height_ = other.maximum_snap_height_;

      min_clearance_from_stance_ = other.min_clearance_from_stance_;

      final_turn_proximity_ = other.final_turn_proximity_;

      yaw_weight_ = other.yaw_weight_;

      pitch_weight_ = other.pitch_weight_;

      roll_weight_ = other.roll_weight_;

      forward_weight_ = other.forward_weight_;

      lateral_weight_ = other.lateral_weight_;

      step_up_weight_ = other.step_up_weight_;

      step_down_weight_ = other.step_down_weight_;

      long_step_weight_ = other.long_step_weight_;

      foothold_area_weight_ = other.foothold_area_weight_;

      cost_per_step_ = other.cost_per_step_;

      a_star_heuristics_weight_ = other.a_star_heuristics_weight_;

      number_of_bounding_box_checks_ = other.number_of_bounding_box_checks_;

      maximum_2d_distance_from_bounding_box_to_penalize_ = other.maximum_2d_distance_from_bounding_box_to_penalize_;

      distance_from_path_tolerance_ = other.distance_from_path_tolerance_;

      delta_yaw_from_reference_tolerance_ = other.delta_yaw_from_reference_tolerance_;

      maximum_branch_factor_ = other.maximum_branch_factor_;

      enable_expansion_mask_ = other.enable_expansion_mask_;

      enable_shin_collision_check_ = other.enable_shin_collision_check_;

      shin_toe_clearance_ = other.shin_toe_clearance_;

      shin_heel_clearance_ = other.shin_heel_clearance_;

      shin_length_ = other.shin_length_;

      shin_height_offet_ = other.shin_height_offet_;

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
            * Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
            * to run slower.
            */
   public void setCheckForBodyBoxCollisions(boolean check_for_body_box_collisions)
   {
      check_for_body_box_collisions_ = check_for_body_box_collisions;
   }
   /**
            * Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
            * to run slower.
            */
   public boolean getCheckForBodyBoxCollisions()
   {
      return check_for_body_box_collisions_;
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
            * Returns the ideal step width for walking on flat ground.
            */
   public void setIdealFootstepWidth(double ideal_footstep_width)
   {
      ideal_footstep_width_ = ideal_footstep_width;
   }
   /**
            * Returns the ideal step width for walking on flat ground.
            */
   public double getIdealFootstepWidth()
   {
      return ideal_footstep_width_;
   }

   /**
            * Returns the ideal step length for walking on flat ground.
            */
   public void setIdealFootstepLength(double ideal_footstep_length)
   {
      ideal_footstep_length_ = ideal_footstep_length;
   }
   /**
            * Returns the ideal step length for walking on flat ground.
            */
   public double getIdealFootstepLength()
   {
      return ideal_footstep_length_;
   }

   /**
            * Returns the ideal step side step width when "shuffling" sideways.
            */
   public void setIdealSideStepWidth(double ideal_side_step_width)
   {
      ideal_side_step_width_ = ideal_side_step_width;
   }
   /**
            * Returns the ideal step side step width when "shuffling" sideways.
            */
   public double getIdealSideStepWidth()
   {
      return ideal_side_step_width_;
   }

   /**
            * Returns the ideal length when walking backwards. This value is positive.
            */
   public void setIdealBackStepLength(double ideal_back_step_length)
   {
      ideal_back_step_length_ = ideal_back_step_length;
   }
   /**
            * Returns the ideal length when walking backwards. This value is positive.
            */
   public double getIdealBackStepLength()
   {
      return ideal_back_step_length_;
   }

   /**
            * Returns ideal step length when the vertical height between the start-of-swing and stance feet are at maximum allowed height.
            */
   public void setIdealStepLengthAtMaxStepZ(double ideal_step_length_at_max_step_z)
   {
      ideal_step_length_at_max_step_z_ = ideal_step_length_at_max_step_z;
   }
   /**
            * Returns ideal step length when the vertical height between the start-of-swing and stance feet are at maximum allowed height.
            */
   public double getIdealStepLengthAtMaxStepZ()
   {
      return ideal_step_length_at_max_step_z_;
   }

   /**
            * The planner will try to shift footsteps inside of a region so that this value is the minimum distance from the step
            * to the edge. A negative value means the footstep can overhang a region.
            */
   public void setWiggleInsideDeltaTarget(double wiggle_inside_delta_target)
   {
      wiggle_inside_delta_target_ = wiggle_inside_delta_target;
   }
   /**
            * The planner will try to shift footsteps inside of a region so that this value is the minimum distance from the step
            * to the edge. A negative value means the footstep can overhang a region.
            */
   public double getWiggleInsideDeltaTarget()
   {
      return wiggle_inside_delta_target_;
   }

   /**
            * This parameter only is used if wiggle_while_planning is true. If a step cannot be wiggled inside by this amount or more,
            * it will be rejected. Note that if {wiggle_while_planning if false, it's always best effort on the final plan.
            */
   public void setWiggleInsideDeltaMinimum(double wiggle_inside_delta_minimum)
   {
      wiggle_inside_delta_minimum_ = wiggle_inside_delta_minimum;
   }
   /**
            * This parameter only is used if wiggle_while_planning is true. If a step cannot be wiggled inside by this amount or more,
            * it will be rejected. Note that if {wiggle_while_planning if false, it's always best effort on the final plan.
            */
   public double getWiggleInsideDeltaMinimum()
   {
      return wiggle_inside_delta_minimum_;
   }

   /**
            * Maximum xy-distance the planner will consider for candidate steps.
            * Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
            */
   public void setMaximumStepReach(double maximum_step_reach)
   {
      maximum_step_reach_ = maximum_step_reach;
   }
   /**
            * Maximum xy-distance the planner will consider for candidate steps.
            * Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
            */
   public double getMaximumStepReach()
   {
      return maximum_step_reach_;
   }

   /**
            * Maximum yaw between consecutive footsteps
            * 
            * A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
            * 
            * This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
            * space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
            * its maximum reach.
            */
   public void setMaximumStepYaw(double maximum_step_yaw)
   {
      maximum_step_yaw_ = maximum_step_yaw;
   }
   /**
            * Maximum yaw between consecutive footsteps
            * 
            * A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
            * 
            * This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
            * space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
            * its maximum reach.
            */
   public double getMaximumStepYaw()
   {
      return maximum_step_yaw_;
   }

   /**
            * Minimum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public void setMinimumStepWidth(double minimum_step_width)
   {
      minimum_step_width_ = minimum_step_width;
   }
   /**
            * Minimum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public double getMinimumStepWidth()
   {
      return minimum_step_width_;
   }

   /**
            * Minimum step length the planner will consider for candidate steps.
            * 
            * Step length refers to the x-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public void setMinimumStepLength(double minimum_step_length)
   {
      minimum_step_length_ = minimum_step_length;
   }
   /**
            * Minimum step length the planner will consider for candidate steps.
            * 
            * Step length refers to the x-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public double getMinimumStepLength()
   {
      return minimum_step_length_;
   }

   /**
            * Minimum step yaw.
            */
   public void setMinimumStepYaw(double minimum_step_yaw)
   {
      minimum_step_yaw_ = minimum_step_yaw;
   }
   /**
            * Minimum step yaw.
            */
   public double getMinimumStepYaw()
   {
      return minimum_step_yaw_;
   }

   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public void setMaximumStepReachWhenSteppingUp(double maximum_step_reach_when_stepping_up)
   {
      maximum_step_reach_when_stepping_up_ = maximum_step_reach_when_stepping_up;
   }
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double getMaximumStepReachWhenSteppingUp()
   {
      return maximum_step_reach_when_stepping_up_;
   }

   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public void setMaximumStepWidthWhenSteppingUp(double maximum_step_width_when_stepping_up)
   {
      maximum_step_width_when_stepping_up_ = maximum_step_width_when_stepping_up;
   }
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double getMaximumStepWidthWhenSteppingUp()
   {
      return maximum_step_width_when_stepping_up_;
   }

   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public void setMaximumStepZWhenSteppingUp(double maximum_step_z_when_stepping_up)
   {
      maximum_step_z_when_stepping_up_ = maximum_step_z_when_stepping_up;
   }
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double getMaximumStepZWhenSteppingUp()
   {
      return maximum_step_z_when_stepping_up_;
   }

   /**
            * Maximum step length when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public void setMaximumStepXWhenForwardAndDown(double maximum_step_x_when_forward_and_down)
   {
      maximum_step_x_when_forward_and_down_ = maximum_step_x_when_forward_and_down;
   }
   /**
            * Maximum step length when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double getMaximumStepXWhenForwardAndDown()
   {
      return maximum_step_x_when_forward_and_down_;
   }

   /**
            * Maximum step width when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public void setMaximumStepYWhenForwardAndDown(double maximum_step_y_when_forward_and_down)
   {
      maximum_step_y_when_forward_and_down_ = maximum_step_y_when_forward_and_down;
   }
   /**
            * Maximum step width when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double getMaximumStepYWhenForwardAndDown()
   {
      return maximum_step_y_when_forward_and_down_;
   }

   /**
            * Maximum step height when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public void setMaximumStepZWhenForwardAndDown(double maximum_step_z_when_forward_and_down)
   {
      maximum_step_z_when_forward_and_down_ = maximum_step_z_when_forward_and_down;
   }
   /**
            * Maximum step height when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if one of two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - OR -
            * The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * - AND -
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double getMaximumStepZWhenForwardAndDown()
   {
      return maximum_step_z_when_forward_and_down_;
   }

   /**
            * Maximum vertical distance between consecutive footsteps
            * 
            * A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
            * z-up sole frame.
            */
   public void setMaximumStepZ(double maximum_step_z)
   {
      maximum_step_z_ = maximum_step_z;
   }
   /**
            * Maximum vertical distance between consecutive footsteps
            * 
            * A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
            * z-up sole frame.
            */
   public double getMaximumStepZ()
   {
      return maximum_step_z_;
   }

   /**
            * Maximum vertical distance between start-of-swing and touchdown
            */
   public void setMaximumSwingZ(double maximum_swing_z)
   {
      maximum_swing_z_ = maximum_swing_z;
   }
   /**
            * Maximum vertical distance between start-of-swing and touchdown
            */
   public double getMaximumSwingZ()
   {
      return maximum_swing_z_;
   }

   /**
            * Maximum xy distance between start-of-swing and touchdown
            */
   public void setMaximumSwingReach(double maximum_swing_reach)
   {
      maximum_swing_reach_ = maximum_swing_reach;
   }
   /**
            * Maximum xy distance between start-of-swing and touchdown
            */
   public double getMaximumSwingReach()
   {
      return maximum_swing_reach_;
   }

   /**
            * Maximum vertical distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
            * 
            * The maximum depth is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
            * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
            */
   public void setMinimumStepZWhenFullyPitched(double minimum_step_z_when_fully_pitched)
   {
      minimum_step_z_when_fully_pitched_ = minimum_step_z_when_fully_pitched;
   }
   /**
            * Maximum vertical distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
            * 
            * The maximum depth is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
            * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
            */
   public double getMinimumStepZWhenFullyPitched()
   {
      return minimum_step_z_when_fully_pitched_;
   }

   /**
            * Maximum forward distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
            * 
            * The maximum distance is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
            * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
            */
   public void setMaximumStepXWhenFullyPitched(double maximum_step_x_when_fully_pitched)
   {
      maximum_step_x_when_fully_pitched_ = maximum_step_x_when_fully_pitched;
   }
   /**
            * Maximum forward distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
            * 
            * The maximum distance is determined by linearly interpolating between {@link #getMaximumStepZ()} and this value, based on the fraction the foot is pitched by.
            * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
            */
   public double getMaximumStepXWhenFullyPitched()
   {
      return maximum_step_x_when_fully_pitched_;
   }

   /**
            * This is the reduction factor for the max yaw when the step is at max reach.
            * This means that, when the footstep is at its maximum distance, this is the fraction reduction of the max yaw.
            * If this returns 0.0, the max yaw is not modified, even at full reach.
            * If this returns 1.0, the max yaw is 0 at full reach.
            * 
            * That is,
            * modifiedMaxYaw = (1.0 - reach / maxReach) * maxYaw + reach / maxReach * (1.0 - alpha) * maxYaw
            */
   public void setStepYawReductionFactorAtMaxReach(double step_yaw_reduction_factor_at_max_reach)
   {
      step_yaw_reduction_factor_at_max_reach_ = step_yaw_reduction_factor_at_max_reach;
   }
   /**
            * This is the reduction factor for the max yaw when the step is at max reach.
            * This means that, when the footstep is at its maximum distance, this is the fraction reduction of the max yaw.
            * If this returns 0.0, the max yaw is not modified, even at full reach.
            * If this returns 1.0, the max yaw is 0 at full reach.
            * 
            * That is,
            * modifiedMaxYaw = (1.0 - reach / maxReach) * maxYaw + reach / maxReach * (1.0 - alpha) * maxYaw
            */
   public double getStepYawReductionFactorAtMaxReach()
   {
      return step_yaw_reduction_factor_at_max_reach_;
   }

   /**
            * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
            * 
            * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
            */
   public void setMinimumFootholdPercent(double minimum_foothold_percent)
   {
      minimum_foothold_percent_ = minimum_foothold_percent;
   }
   /**
            * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
            * 
            * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
            */
   public double getMinimumFootholdPercent()
   {
      return minimum_foothold_percent_;
   }

   /**
            * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
            * then the value specified here.
            * 
            * More specifically, if a footstep has an associated planar region and that regions surface normal has a
            * z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
            */
   public void setMinimumSurfaceInclineRadians(double minimum_surface_incline_radians)
   {
      minimum_surface_incline_radians_ = minimum_surface_incline_radians;
   }
   /**
            * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
            * then the value specified here.
            * 
            * More specifically, if a footstep has an associated planar region and that regions surface normal has a
            * z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
            */
   public double getMinimumSurfaceInclineRadians()
   {
      return minimum_surface_incline_radians_;
   }

   /**
            * The wiggler can either run as a post-processor on a resulting plan or on each candidate step while planning.
            * If true, this wiggles each candidate step, which will slow down plan times but resulting plans will be guarunteed to match step constraints.
            */
   public void setWiggleWhilePlanning(boolean wiggle_while_planning)
   {
      wiggle_while_planning_ = wiggle_while_planning;
   }
   /**
            * The wiggler can either run as a post-processor on a resulting plan or on each candidate step while planning.
            * If true, this wiggles each candidate step, which will slow down plan times but resulting plans will be guarunteed to match step constraints.
            */
   public boolean getWiggleWhilePlanning()
   {
      return wiggle_while_planning_;
   }

   /**
            * If wiggle_while_planning is true, this will reject a step if the wiggle meet the specified parameters. If it's false the wiggle does a best effort
            */
   public void setRejectIfWiggleNotSatisfied(boolean reject_if_wiggle_not_satisfied)
   {
      reject_if_wiggle_not_satisfied_ = reject_if_wiggle_not_satisfied;
   }
   /**
            * If wiggle_while_planning is true, this will reject a step if the wiggle meet the specified parameters. If it's false the wiggle does a best effort
            */
   public boolean getRejectIfWiggleNotSatisfied()
   {
      return reject_if_wiggle_not_satisfied_;
   }

   /**
            * There are two solvers for wiggling the step, one constrains to the region's convex hull and the other to the region's concave hull,
            * this toggles between them.
            */
   public void setEnableConcaveHullWiggler(boolean enable_concave_hull_wiggler)
   {
      enable_concave_hull_wiggler_ = enable_concave_hull_wiggler;
   }
   /**
            * There are two solvers for wiggling the step, one constrains to the region's convex hull and the other to the region's concave hull,
            * this toggles between them.
            */
   public boolean getEnableConcaveHullWiggler()
   {
      return enable_concave_hull_wiggler_;
   }

   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
            * distance the planner will use
            */
   public void setMaximumXyWiggleDistance(double maximum_xy_wiggle_distance)
   {
      maximum_xy_wiggle_distance_ = maximum_xy_wiggle_distance;
   }
   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
            * distance the planner will use
            */
   public double getMaximumXyWiggleDistance()
   {
      return maximum_xy_wiggle_distance_;
   }

   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum yaw
            * distance the planner will use
            */
   public void setMaximumYawWiggle(double maximum_yaw_wiggle)
   {
      maximum_yaw_wiggle_ = maximum_yaw_wiggle;
   }
   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum yaw
            * distance the planner will use
            */
   public double getMaximumYawWiggle()
   {
      return maximum_yaw_wiggle_;
   }

   /**
            * When snapping a candidate footstep to a planar region, its possible that another planar region
            * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
            * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
            * otherwise it will.
            */
   public void setMaximumZPenetrationOnValleyRegions(double maximum_z_penetration_on_valley_regions)
   {
      maximum_z_penetration_on_valley_regions_ = maximum_z_penetration_on_valley_regions;
   }
   /**
            * When snapping a candidate footstep to a planar region, its possible that another planar region
            * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
            * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
            * otherwise it will.
            */
   public double getMaximumZPenetrationOnValleyRegions()
   {
      return maximum_z_penetration_on_valley_regions_;
   }

   /**
            * Maximum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
            */
   public void setMaximumStepWidth(double maximum_step_width)
   {
      maximum_step_width_ = maximum_step_width;
   }
   /**
            * Maximum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
            */
   public double getMaximumStepWidth()
   {
      return maximum_step_width_;
   }

   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public void setCliffBaseHeightToAvoid(double cliff_base_height_to_avoid)
   {
      cliff_base_height_to_avoid_ = cliff_base_height_to_avoid;
   }
   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double getCliffBaseHeightToAvoid()
   {
      return cliff_base_height_to_avoid_;
   }

   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffBaseHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public void setMinimumDistanceFromCliffBottoms(double minimum_distance_from_cliff_bottoms)
   {
      minimum_distance_from_cliff_bottoms_ = minimum_distance_from_cliff_bottoms;
   }
   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffBaseHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimum_distance_from_cliff_bottoms_;
   }

   /**
            * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffTops away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public void setCliffTopHeightToAvoid(double cliff_top_height_to_avoid)
   {
      cliff_top_height_to_avoid_ = cliff_top_height_to_avoid;
   }
   /**
            * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffTops away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double getCliffTopHeightToAvoid()
   {
      return cliff_top_height_to_avoid_;
   }

   /**
            * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffTopHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public void setMinimumDistanceFromCliffTops(double minimum_distance_from_cliff_tops)
   {
      minimum_distance_from_cliff_tops_ = minimum_distance_from_cliff_tops;
   }
   /**
            * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffTopHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double getMinimumDistanceFromCliffTops()
   {
      return minimum_distance_from_cliff_tops_;
   }

   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box height.
            */
   public void setBodyBoxHeight(double body_box_height)
   {
      body_box_height_ = body_box_height;
   }
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box height.
            */
   public double getBodyBoxHeight()
   {
      return body_box_height_;
   }

   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box depth.
            */
   public void setBodyBoxDepth(double body_box_depth)
   {
      body_box_depth_ = body_box_depth;
   }
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box depth.
            */
   public double getBodyBoxDepth()
   {
      return body_box_depth_;
   }

   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box width.
            */
   public void setBodyBoxWidth(double body_box_width)
   {
      body_box_width_ = body_box_width;
   }
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box width.
            */
   public double getBodyBoxWidth()
   {
      return body_box_width_;
   }

   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public void setBodyBoxBaseX(double body_box_base_x)
   {
      body_box_base_x_ = body_box_base_x;
   }
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double getBodyBoxBaseX()
   {
      return body_box_base_x_;
   }

   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public void setBodyBoxBaseY(double body_box_base_y)
   {
      body_box_base_y_ = body_box_base_y;
   }
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double getBodyBoxBaseY()
   {
      return body_box_base_y_;
   }

   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public void setBodyBoxBaseZ(double body_box_base_z)
   {
      body_box_base_z_ = body_box_base_z;
   }
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double getBodyBoxBaseZ()
   {
      return body_box_base_z_;
   }

   /**
            * Maximum height above a stance step that a candidate step is snapped to. Regions above this height are ignored.
            * Intended to avoid ceilings or obstacles that are above the top of the robot
            */
   public void setMaximumSnapHeight(double maximum_snap_height)
   {
      maximum_snap_height_ = maximum_snap_height;
   }
   /**
            * Maximum height above a stance step that a candidate step is snapped to. Regions above this height are ignored.
            * Intended to avoid ceilings or obstacles that are above the top of the robot
            */
   public double getMaximumSnapHeight()
   {
      return maximum_snap_height_;
   }

   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public void setMinClearanceFromStance(double min_clearance_from_stance)
   {
      min_clearance_from_stance_ = min_clearance_from_stance;
   }
   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public double getMinClearanceFromStance()
   {
      return min_clearance_from_stance_;
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
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public void setYawWeight(double yaw_weight)
   {
      yaw_weight_ = yaw_weight;
   }
   /**
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public double getYawWeight()
   {
      return yaw_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public void setPitchWeight(double pitch_weight)
   {
      pitch_weight_ = pitch_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public double getPitchWeight()
   {
      return pitch_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public void setRollWeight(double roll_weight)
   {
      roll_weight_ = roll_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public double getRollWeight()
   {
      return roll_weight_;
   }

   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public void setForwardWeight(double forward_weight)
   {
      forward_weight_ = forward_weight;
   }
   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public double getForwardWeight()
   {
      return forward_weight_;
   }

   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public void setLateralWeight(double lateral_weight)
   {
      lateral_weight_ = lateral_weight;
   }
   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public double getLateralWeight()
   {
      return lateral_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public void setStepUpWeight(double step_up_weight)
   {
      step_up_weight_ = step_up_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public double getStepUpWeight()
   {
      return step_up_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public void setStepDownWeight(double step_down_weight)
   {
      step_down_weight_ = step_down_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public double getStepDownWeight()
   {
      return step_down_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public void setLongStepWeight(double long_step_weight)
   {
      long_step_weight_ = long_step_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public double getLongStepWeight()
   {
      return long_step_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public void setFootholdAreaWeight(double foothold_area_weight)
   {
      foothold_area_weight_ = foothold_area_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public double getFootholdAreaWeight()
   {
      return foothold_area_weight_;
   }

   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public void setCostPerStep(double cost_per_step)
   {
      cost_per_step_ = cost_per_step;
   }
   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public double getCostPerStep()
   {
      return cost_per_step_;
   }

   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public void setAStarHeuristicsWeight(double a_star_heuristics_weight)
   {
      a_star_heuristics_weight_ = a_star_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public double getAStarHeuristicsWeight()
   {
      return a_star_heuristics_weight_;
   }

   /**
            * This sets how many bounding box checks to perform. If this value is 1, only the final footstep is checked.
            * Additional checks are done by interpolating between the start and end steps
            */
   public void setNumberOfBoundingBoxChecks(long number_of_bounding_box_checks)
   {
      number_of_bounding_box_checks_ = number_of_bounding_box_checks;
   }
   /**
            * This sets how many bounding box checks to perform. If this value is 1, only the final footstep is checked.
            * Additional checks are done by interpolating between the start and end steps
            */
   public long getNumberOfBoundingBoxChecks()
   {
      return number_of_bounding_box_checks_;
   }

   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum_2d_distance_from_bounding_box_to_penalize)
   {
      maximum_2d_distance_from_bounding_box_to_penalize_ = maximum_2d_distance_from_bounding_box_to_penalize;
   }
   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return maximum_2d_distance_from_bounding_box_to_penalize_;
   }

   /**
            * If the robot's mid-foot pose is within this distance of the body path, it will match the body path heading.
            * Otherwise, it will turn towards the body path
            */
   public void setDistanceFromPathTolerance(double distance_from_path_tolerance)
   {
      distance_from_path_tolerance_ = distance_from_path_tolerance;
   }
   /**
            * If the robot's mid-foot pose is within this distance of the body path, it will match the body path heading.
            * Otherwise, it will turn towards the body path
            */
   public double getDistanceFromPathTolerance()
   {
      return distance_from_path_tolerance_;
   }

   /**
            * If the robot's mid-foot pose oriented within this threshold of the body path's heading, it will match the body path heading.
            * Otherwise, it will turn in plance towards the body path
            */
   public void setDeltaYawFromReferenceTolerance(double delta_yaw_from_reference_tolerance)
   {
      delta_yaw_from_reference_tolerance_ = delta_yaw_from_reference_tolerance;
   }
   /**
            * If the robot's mid-foot pose oriented within this threshold of the body path's heading, it will match the body path heading.
            * Otherwise, it will turn in plance towards the body path
            */
   public double getDeltaYawFromReferenceTolerance()
   {
      return delta_yaw_from_reference_tolerance_;
   }

   /**
            * Maximum steps considered at each iteration. If more than this number of steps are available, the closest steps to the
            * ideal step are considered and the others are ignored. Set to non-positive number to disable
            */
   public void setMaximumBranchFactor(int maximum_branch_factor)
   {
      maximum_branch_factor_ = maximum_branch_factor;
   }
   /**
            * Maximum steps considered at each iteration. If more than this number of steps are available, the closest steps to the
            * ideal step are considered and the others are ignored. Set to non-positive number to disable
            */
   public int getMaximumBranchFactor()
   {
      return maximum_branch_factor_;
   }

   /**
            * If true, enables a mask that reduces the number of calculated steps away from the ideal step
            */
   public void setEnableExpansionMask(boolean enable_expansion_mask)
   {
      enable_expansion_mask_ = enable_expansion_mask;
   }
   /**
            * If true, enables a mask that reduces the number of calculated steps away from the ideal step
            */
   public boolean getEnableExpansionMask()
   {
      return enable_expansion_mask_;
   }

   /**
            * If true will try to wiggle steps away from shin collisions. Collisions are checked against all regions.
            * Enable concave hull wiggler must be true in order for the shin collision checker to run.
            */
   public void setEnableShinCollisionCheck(boolean enable_shin_collision_check)
   {
      enable_shin_collision_check_ = enable_shin_collision_check;
   }
   /**
            * If true will try to wiggle steps away from shin collisions. Collisions are checked against all regions.
            * Enable concave hull wiggler must be true in order for the shin collision checker to run.
            */
   public boolean getEnableShinCollisionCheck()
   {
      return enable_shin_collision_check_;
   }

   /**
            * How far the shin collision cylinder extends from the toe
            */
   public void setShinToeClearance(double shin_toe_clearance)
   {
      shin_toe_clearance_ = shin_toe_clearance;
   }
   /**
            * How far the shin collision cylinder extends from the toe
            */
   public double getShinToeClearance()
   {
      return shin_toe_clearance_;
   }

   /**
            * How far the shin collision cylinder extends from the heel
            */
   public void setShinHeelClearance(double shin_heel_clearance)
   {
      shin_heel_clearance_ = shin_heel_clearance;
   }
   /**
            * How far the shin collision cylinder extends from the heel
            */
   public double getShinHeelClearance()
   {
      return shin_heel_clearance_;
   }

   /**
            * Length of the shin collidable cylinder
            */
   public void setShinLength(double shin_length)
   {
      shin_length_ = shin_length;
   }
   /**
            * Length of the shin collidable cylinder
            */
   public double getShinLength()
   {
      return shin_length_;
   }

   /**
            * Height offset of shin collidable cylinder
            */
   public void setShinHeightOffet(double shin_height_offet)
   {
      shin_height_offet_ = shin_height_offet;
   }
   /**
            * Height offset of shin collidable cylinder
            */
   public double getShinHeightOffet()
   {
      return shin_height_offet_;
   }


   public static Supplier<FootstepPlannerParametersPacketPubSubType> getPubSubType()
   {
      return FootstepPlannerParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_for_body_box_collisions_, other.check_for_body_box_collisions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_for_path_collisions_, other.check_for_path_collisions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_footstep_width_, other.ideal_footstep_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_footstep_length_, other.ideal_footstep_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_side_step_width_, other.ideal_side_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_back_step_length_, other.ideal_back_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_step_length_at_max_step_z_, other.ideal_step_length_at_max_step_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wiggle_inside_delta_target_, other.wiggle_inside_delta_target_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wiggle_inside_delta_minimum_, other.wiggle_inside_delta_minimum_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_, other.maximum_step_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_yaw_, other.maximum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_width_, other.minimum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_length_, other.minimum_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_yaw_, other.minimum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_when_stepping_up_, other.maximum_step_reach_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_when_stepping_up_, other.maximum_step_width_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_when_stepping_up_, other.maximum_step_z_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_x_when_forward_and_down_, other.maximum_step_x_when_forward_and_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_y_when_forward_and_down_, other.maximum_step_y_when_forward_and_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_when_forward_and_down_, other.maximum_step_z_when_forward_and_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_, other.maximum_step_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_swing_z_, other.maximum_swing_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_swing_reach_, other.maximum_swing_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_z_when_fully_pitched_, other.minimum_step_z_when_fully_pitched_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_x_when_fully_pitched_, other.maximum_step_x_when_fully_pitched_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_yaw_reduction_factor_at_max_reach_, other.step_yaw_reduction_factor_at_max_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_foothold_percent_, other.minimum_foothold_percent_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_surface_incline_radians_, other.minimum_surface_incline_radians_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.wiggle_while_planning_, other.wiggle_while_planning_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.reject_if_wiggle_not_satisfied_, other.reject_if_wiggle_not_satisfied_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_concave_hull_wiggler_, other.enable_concave_hull_wiggler_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_xy_wiggle_distance_, other.maximum_xy_wiggle_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_yaw_wiggle_, other.maximum_yaw_wiggle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_z_penetration_on_valley_regions_, other.maximum_z_penetration_on_valley_regions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_, other.maximum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cliff_base_height_to_avoid_, other.cliff_base_height_to_avoid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_distance_from_cliff_bottoms_, other.minimum_distance_from_cliff_bottoms_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cliff_top_height_to_avoid_, other.cliff_top_height_to_avoid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_distance_from_cliff_tops_, other.minimum_distance_from_cliff_tops_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_height_, other.body_box_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_depth_, other.body_box_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_width_, other.body_box_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_base_x_, other.body_box_base_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_base_y_, other.body_box_base_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_base_z_, other.body_box_base_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_snap_height_, other.maximum_snap_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_clearance_from_stance_, other.min_clearance_from_stance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_turn_proximity_, other.final_turn_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_weight_, other.yaw_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pitch_weight_, other.pitch_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.roll_weight_, other.roll_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_weight_, other.forward_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lateral_weight_, other.lateral_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_up_weight_, other.step_up_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_down_weight_, other.step_down_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.long_step_weight_, other.long_step_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foothold_area_weight_, other.foothold_area_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cost_per_step_, other.cost_per_step_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.a_star_heuristics_weight_, other.a_star_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_bounding_box_checks_, other.number_of_bounding_box_checks_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_2d_distance_from_bounding_box_to_penalize_, other.maximum_2d_distance_from_bounding_box_to_penalize_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_from_path_tolerance_, other.distance_from_path_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.delta_yaw_from_reference_tolerance_, other.delta_yaw_from_reference_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_branch_factor_, other.maximum_branch_factor_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_expansion_mask_, other.enable_expansion_mask_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_shin_collision_check_, other.enable_shin_collision_check_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shin_toe_clearance_, other.shin_toe_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shin_heel_clearance_, other.shin_heel_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shin_length_, other.shin_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shin_height_offet_, other.shin_height_offet_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerParametersPacket)) return false;

      FootstepPlannerParametersPacket otherMyClass = (FootstepPlannerParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.check_for_body_box_collisions_ != otherMyClass.check_for_body_box_collisions_) return false;

      if(this.check_for_path_collisions_ != otherMyClass.check_for_path_collisions_) return false;

      if(this.ideal_footstep_width_ != otherMyClass.ideal_footstep_width_) return false;

      if(this.ideal_footstep_length_ != otherMyClass.ideal_footstep_length_) return false;

      if(this.ideal_side_step_width_ != otherMyClass.ideal_side_step_width_) return false;

      if(this.ideal_back_step_length_ != otherMyClass.ideal_back_step_length_) return false;

      if(this.ideal_step_length_at_max_step_z_ != otherMyClass.ideal_step_length_at_max_step_z_) return false;

      if(this.wiggle_inside_delta_target_ != otherMyClass.wiggle_inside_delta_target_) return false;

      if(this.wiggle_inside_delta_minimum_ != otherMyClass.wiggle_inside_delta_minimum_) return false;

      if(this.maximum_step_reach_ != otherMyClass.maximum_step_reach_) return false;

      if(this.maximum_step_yaw_ != otherMyClass.maximum_step_yaw_) return false;

      if(this.minimum_step_width_ != otherMyClass.minimum_step_width_) return false;

      if(this.minimum_step_length_ != otherMyClass.minimum_step_length_) return false;

      if(this.minimum_step_yaw_ != otherMyClass.minimum_step_yaw_) return false;

      if(this.maximum_step_reach_when_stepping_up_ != otherMyClass.maximum_step_reach_when_stepping_up_) return false;

      if(this.maximum_step_width_when_stepping_up_ != otherMyClass.maximum_step_width_when_stepping_up_) return false;

      if(this.maximum_step_z_when_stepping_up_ != otherMyClass.maximum_step_z_when_stepping_up_) return false;

      if(this.maximum_step_x_when_forward_and_down_ != otherMyClass.maximum_step_x_when_forward_and_down_) return false;

      if(this.maximum_step_y_when_forward_and_down_ != otherMyClass.maximum_step_y_when_forward_and_down_) return false;

      if(this.maximum_step_z_when_forward_and_down_ != otherMyClass.maximum_step_z_when_forward_and_down_) return false;

      if(this.maximum_step_z_ != otherMyClass.maximum_step_z_) return false;

      if(this.maximum_swing_z_ != otherMyClass.maximum_swing_z_) return false;

      if(this.maximum_swing_reach_ != otherMyClass.maximum_swing_reach_) return false;

      if(this.minimum_step_z_when_fully_pitched_ != otherMyClass.minimum_step_z_when_fully_pitched_) return false;

      if(this.maximum_step_x_when_fully_pitched_ != otherMyClass.maximum_step_x_when_fully_pitched_) return false;

      if(this.step_yaw_reduction_factor_at_max_reach_ != otherMyClass.step_yaw_reduction_factor_at_max_reach_) return false;

      if(this.minimum_foothold_percent_ != otherMyClass.minimum_foothold_percent_) return false;

      if(this.minimum_surface_incline_radians_ != otherMyClass.minimum_surface_incline_radians_) return false;

      if(this.wiggle_while_planning_ != otherMyClass.wiggle_while_planning_) return false;

      if(this.reject_if_wiggle_not_satisfied_ != otherMyClass.reject_if_wiggle_not_satisfied_) return false;

      if(this.enable_concave_hull_wiggler_ != otherMyClass.enable_concave_hull_wiggler_) return false;

      if(this.maximum_xy_wiggle_distance_ != otherMyClass.maximum_xy_wiggle_distance_) return false;

      if(this.maximum_yaw_wiggle_ != otherMyClass.maximum_yaw_wiggle_) return false;

      if(this.maximum_z_penetration_on_valley_regions_ != otherMyClass.maximum_z_penetration_on_valley_regions_) return false;

      if(this.maximum_step_width_ != otherMyClass.maximum_step_width_) return false;

      if(this.cliff_base_height_to_avoid_ != otherMyClass.cliff_base_height_to_avoid_) return false;

      if(this.minimum_distance_from_cliff_bottoms_ != otherMyClass.minimum_distance_from_cliff_bottoms_) return false;

      if(this.cliff_top_height_to_avoid_ != otherMyClass.cliff_top_height_to_avoid_) return false;

      if(this.minimum_distance_from_cliff_tops_ != otherMyClass.minimum_distance_from_cliff_tops_) return false;

      if(this.body_box_height_ != otherMyClass.body_box_height_) return false;

      if(this.body_box_depth_ != otherMyClass.body_box_depth_) return false;

      if(this.body_box_width_ != otherMyClass.body_box_width_) return false;

      if(this.body_box_base_x_ != otherMyClass.body_box_base_x_) return false;

      if(this.body_box_base_y_ != otherMyClass.body_box_base_y_) return false;

      if(this.body_box_base_z_ != otherMyClass.body_box_base_z_) return false;

      if(this.maximum_snap_height_ != otherMyClass.maximum_snap_height_) return false;

      if(this.min_clearance_from_stance_ != otherMyClass.min_clearance_from_stance_) return false;

      if(this.final_turn_proximity_ != otherMyClass.final_turn_proximity_) return false;

      if(this.yaw_weight_ != otherMyClass.yaw_weight_) return false;

      if(this.pitch_weight_ != otherMyClass.pitch_weight_) return false;

      if(this.roll_weight_ != otherMyClass.roll_weight_) return false;

      if(this.forward_weight_ != otherMyClass.forward_weight_) return false;

      if(this.lateral_weight_ != otherMyClass.lateral_weight_) return false;

      if(this.step_up_weight_ != otherMyClass.step_up_weight_) return false;

      if(this.step_down_weight_ != otherMyClass.step_down_weight_) return false;

      if(this.long_step_weight_ != otherMyClass.long_step_weight_) return false;

      if(this.foothold_area_weight_ != otherMyClass.foothold_area_weight_) return false;

      if(this.cost_per_step_ != otherMyClass.cost_per_step_) return false;

      if(this.a_star_heuristics_weight_ != otherMyClass.a_star_heuristics_weight_) return false;

      if(this.number_of_bounding_box_checks_ != otherMyClass.number_of_bounding_box_checks_) return false;

      if(this.maximum_2d_distance_from_bounding_box_to_penalize_ != otherMyClass.maximum_2d_distance_from_bounding_box_to_penalize_) return false;

      if(this.distance_from_path_tolerance_ != otherMyClass.distance_from_path_tolerance_) return false;

      if(this.delta_yaw_from_reference_tolerance_ != otherMyClass.delta_yaw_from_reference_tolerance_) return false;

      if(this.maximum_branch_factor_ != otherMyClass.maximum_branch_factor_) return false;

      if(this.enable_expansion_mask_ != otherMyClass.enable_expansion_mask_) return false;

      if(this.enable_shin_collision_check_ != otherMyClass.enable_shin_collision_check_) return false;

      if(this.shin_toe_clearance_ != otherMyClass.shin_toe_clearance_) return false;

      if(this.shin_heel_clearance_ != otherMyClass.shin_heel_clearance_) return false;

      if(this.shin_length_ != otherMyClass.shin_length_) return false;

      if(this.shin_height_offet_ != otherMyClass.shin_height_offet_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("check_for_body_box_collisions=");
      builder.append(this.check_for_body_box_collisions_);      builder.append(", ");
      builder.append("check_for_path_collisions=");
      builder.append(this.check_for_path_collisions_);      builder.append(", ");
      builder.append("ideal_footstep_width=");
      builder.append(this.ideal_footstep_width_);      builder.append(", ");
      builder.append("ideal_footstep_length=");
      builder.append(this.ideal_footstep_length_);      builder.append(", ");
      builder.append("ideal_side_step_width=");
      builder.append(this.ideal_side_step_width_);      builder.append(", ");
      builder.append("ideal_back_step_length=");
      builder.append(this.ideal_back_step_length_);      builder.append(", ");
      builder.append("ideal_step_length_at_max_step_z=");
      builder.append(this.ideal_step_length_at_max_step_z_);      builder.append(", ");
      builder.append("wiggle_inside_delta_target=");
      builder.append(this.wiggle_inside_delta_target_);      builder.append(", ");
      builder.append("wiggle_inside_delta_minimum=");
      builder.append(this.wiggle_inside_delta_minimum_);      builder.append(", ");
      builder.append("maximum_step_reach=");
      builder.append(this.maximum_step_reach_);      builder.append(", ");
      builder.append("maximum_step_yaw=");
      builder.append(this.maximum_step_yaw_);      builder.append(", ");
      builder.append("minimum_step_width=");
      builder.append(this.minimum_step_width_);      builder.append(", ");
      builder.append("minimum_step_length=");
      builder.append(this.minimum_step_length_);      builder.append(", ");
      builder.append("minimum_step_yaw=");
      builder.append(this.minimum_step_yaw_);      builder.append(", ");
      builder.append("maximum_step_reach_when_stepping_up=");
      builder.append(this.maximum_step_reach_when_stepping_up_);      builder.append(", ");
      builder.append("maximum_step_width_when_stepping_up=");
      builder.append(this.maximum_step_width_when_stepping_up_);      builder.append(", ");
      builder.append("maximum_step_z_when_stepping_up=");
      builder.append(this.maximum_step_z_when_stepping_up_);      builder.append(", ");
      builder.append("maximum_step_x_when_forward_and_down=");
      builder.append(this.maximum_step_x_when_forward_and_down_);      builder.append(", ");
      builder.append("maximum_step_y_when_forward_and_down=");
      builder.append(this.maximum_step_y_when_forward_and_down_);      builder.append(", ");
      builder.append("maximum_step_z_when_forward_and_down=");
      builder.append(this.maximum_step_z_when_forward_and_down_);      builder.append(", ");
      builder.append("maximum_step_z=");
      builder.append(this.maximum_step_z_);      builder.append(", ");
      builder.append("maximum_swing_z=");
      builder.append(this.maximum_swing_z_);      builder.append(", ");
      builder.append("maximum_swing_reach=");
      builder.append(this.maximum_swing_reach_);      builder.append(", ");
      builder.append("minimum_step_z_when_fully_pitched=");
      builder.append(this.minimum_step_z_when_fully_pitched_);      builder.append(", ");
      builder.append("maximum_step_x_when_fully_pitched=");
      builder.append(this.maximum_step_x_when_fully_pitched_);      builder.append(", ");
      builder.append("step_yaw_reduction_factor_at_max_reach=");
      builder.append(this.step_yaw_reduction_factor_at_max_reach_);      builder.append(", ");
      builder.append("minimum_foothold_percent=");
      builder.append(this.minimum_foothold_percent_);      builder.append(", ");
      builder.append("minimum_surface_incline_radians=");
      builder.append(this.minimum_surface_incline_radians_);      builder.append(", ");
      builder.append("wiggle_while_planning=");
      builder.append(this.wiggle_while_planning_);      builder.append(", ");
      builder.append("reject_if_wiggle_not_satisfied=");
      builder.append(this.reject_if_wiggle_not_satisfied_);      builder.append(", ");
      builder.append("enable_concave_hull_wiggler=");
      builder.append(this.enable_concave_hull_wiggler_);      builder.append(", ");
      builder.append("maximum_xy_wiggle_distance=");
      builder.append(this.maximum_xy_wiggle_distance_);      builder.append(", ");
      builder.append("maximum_yaw_wiggle=");
      builder.append(this.maximum_yaw_wiggle_);      builder.append(", ");
      builder.append("maximum_z_penetration_on_valley_regions=");
      builder.append(this.maximum_z_penetration_on_valley_regions_);      builder.append(", ");
      builder.append("maximum_step_width=");
      builder.append(this.maximum_step_width_);      builder.append(", ");
      builder.append("cliff_base_height_to_avoid=");
      builder.append(this.cliff_base_height_to_avoid_);      builder.append(", ");
      builder.append("minimum_distance_from_cliff_bottoms=");
      builder.append(this.minimum_distance_from_cliff_bottoms_);      builder.append(", ");
      builder.append("cliff_top_height_to_avoid=");
      builder.append(this.cliff_top_height_to_avoid_);      builder.append(", ");
      builder.append("minimum_distance_from_cliff_tops=");
      builder.append(this.minimum_distance_from_cliff_tops_);      builder.append(", ");
      builder.append("body_box_height=");
      builder.append(this.body_box_height_);      builder.append(", ");
      builder.append("body_box_depth=");
      builder.append(this.body_box_depth_);      builder.append(", ");
      builder.append("body_box_width=");
      builder.append(this.body_box_width_);      builder.append(", ");
      builder.append("body_box_base_x=");
      builder.append(this.body_box_base_x_);      builder.append(", ");
      builder.append("body_box_base_y=");
      builder.append(this.body_box_base_y_);      builder.append(", ");
      builder.append("body_box_base_z=");
      builder.append(this.body_box_base_z_);      builder.append(", ");
      builder.append("maximum_snap_height=");
      builder.append(this.maximum_snap_height_);      builder.append(", ");
      builder.append("min_clearance_from_stance=");
      builder.append(this.min_clearance_from_stance_);      builder.append(", ");
      builder.append("final_turn_proximity=");
      builder.append(this.final_turn_proximity_);      builder.append(", ");
      builder.append("yaw_weight=");
      builder.append(this.yaw_weight_);      builder.append(", ");
      builder.append("pitch_weight=");
      builder.append(this.pitch_weight_);      builder.append(", ");
      builder.append("roll_weight=");
      builder.append(this.roll_weight_);      builder.append(", ");
      builder.append("forward_weight=");
      builder.append(this.forward_weight_);      builder.append(", ");
      builder.append("lateral_weight=");
      builder.append(this.lateral_weight_);      builder.append(", ");
      builder.append("step_up_weight=");
      builder.append(this.step_up_weight_);      builder.append(", ");
      builder.append("step_down_weight=");
      builder.append(this.step_down_weight_);      builder.append(", ");
      builder.append("long_step_weight=");
      builder.append(this.long_step_weight_);      builder.append(", ");
      builder.append("foothold_area_weight=");
      builder.append(this.foothold_area_weight_);      builder.append(", ");
      builder.append("cost_per_step=");
      builder.append(this.cost_per_step_);      builder.append(", ");
      builder.append("a_star_heuristics_weight=");
      builder.append(this.a_star_heuristics_weight_);      builder.append(", ");
      builder.append("number_of_bounding_box_checks=");
      builder.append(this.number_of_bounding_box_checks_);      builder.append(", ");
      builder.append("maximum_2d_distance_from_bounding_box_to_penalize=");
      builder.append(this.maximum_2d_distance_from_bounding_box_to_penalize_);      builder.append(", ");
      builder.append("distance_from_path_tolerance=");
      builder.append(this.distance_from_path_tolerance_);      builder.append(", ");
      builder.append("delta_yaw_from_reference_tolerance=");
      builder.append(this.delta_yaw_from_reference_tolerance_);      builder.append(", ");
      builder.append("maximum_branch_factor=");
      builder.append(this.maximum_branch_factor_);      builder.append(", ");
      builder.append("enable_expansion_mask=");
      builder.append(this.enable_expansion_mask_);      builder.append(", ");
      builder.append("enable_shin_collision_check=");
      builder.append(this.enable_shin_collision_check_);      builder.append(", ");
      builder.append("shin_toe_clearance=");
      builder.append(this.shin_toe_clearance_);      builder.append(", ");
      builder.append("shin_heel_clearance=");
      builder.append(this.shin_heel_clearance_);      builder.append(", ");
      builder.append("shin_length=");
      builder.append(this.shin_length_);      builder.append(", ");
      builder.append("shin_height_offet=");
      builder.append(this.shin_height_offet_);
      builder.append("}");
      return builder.toString();
   }
}
