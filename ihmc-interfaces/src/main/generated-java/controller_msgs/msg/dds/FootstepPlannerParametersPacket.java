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
       * Long steps forward are rejected by the planner if two criteria are met:
       * The x-position of the value of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * Step height for considering stepping up.
       * 
       * Long steps forward are rejected by the planner if two criteria are met:
       * The x-position of the value of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       * The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame.
       */
public class FootstepPlannerParametersPacket extends Packet<FootstepPlannerParametersPacket> implements Settable<FootstepPlannerParametersPacket>, EpsilonComparable<FootstepPlannerParametersPacket>
{
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
            * Sets whether or not to perform the defined heuristic search policies.
            */
   public boolean perform_heuristic_search_policies_;
   /**
            * Returns the ideal step width for walking on flat ground.
            */
   public double ideal_footstep_width_ = -1.0;
   /**
            * Returns the ideal step length for walking on flat ground.
            */
   public double ideal_footstep_length_ = -1.0;
   /**
            * If the planner in use utilized footstep wiggling (see {@link PolygonWiggler}) to move footholds onto planer
            * regions this parameter will be used. It specifies the minimum distance between the foot polygon and the
            * edge of the planar region polygon that the footstep is moved into. This value can be negative. That corresponds
            * to allowing footsteps that partially intersect planar regions.
            * 
            * If this value is too high, the planner will not put footsteps on small planar regions. At zero, the planner might
            * choose a footstep with an edge along a planar region. This value should roughly be set to the sum of two values:
            * The smallest acceptable distance to the edge of a cliff
            * The maximum error between desired and actual foot placement
            */
   public double wiggle_inside_delta_;
   /**
            * Maximum xy-distance the planner will consider for candidate steps.
            * Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
            */
   public double maximum_step_reach_ = -1.0;
   /**
            * Maximum yaw between consecutive footsteps
            * 
            * A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
            * 
            * This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
            * space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
            * its maximum reach.
            */
   public double maximum_step_yaw_ = -1.0;
   /**
            * Minimum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public double minimum_step_width_ = -10.0;
   /**
            * Minimum step length the planner will consider for candidate steps.
            * 
            * Step length refers to the x-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
            * on top of each other. If too high, footsteps might not be kinematically feasible.
            */
   public double minimum_step_length_ = -1.0;
   /**
            * Minimum step yaw.
            */
   public double minimum_step_yaw_ = -1.0;
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double maximum_step_reach_when_stepping_up_ = -1.0;
   /**
            * Large steps forward and up can cause the robot to surpass its torque limits.
            * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
            * it's very close to saturating its torque limits.
            */
   public double maximum_step_z_when_stepping_up_ = -1.0;
   /**
            * Maximum step length when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double maximum_step_x_when_forward_and_down_ = -1.0;
   /**
            * Maximum step height when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * The z-position of the value of the footstep is less than -maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * 
            * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
            * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
            * it's very close to hitting it's ankle pitch joint limit.
            */
   public double maximum_step_z_when_forward_and_down_ = -1.0;
   /**
            * Maximum vertical distance between consecutive footsteps
            * 
            * A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
            * z-up sole frame.
            */
   public double maximum_step_z_ = -1.0;
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
   public double minimum_surface_incline_radians_ = -1.0;
   /**
            * There are two methods of wiggling a polygon into a planar region:
            * Wiggle the polygon into the planar region itself, which isn't necessarily convex
            * Wiggle the polygon into the convex hull of the planar region
            * The first method is not implemented completely. Instead it will wiggle into the sub polygon of the planar region that
            * has the biggest overlap with the foothold.
            * 
            * If this parameter is set to true (recommended), the second wiggle method will be used.
            */
   public boolean wiggle_into_convex_hull_of_planar_regions_ = true;
   /**
            * If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
            * This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
            * {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
            * candidate footstep will be rejected if this method returns {@code true}.
            */
   public boolean reject_if_cannot_fully_wiggle_inside_;
   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
            * distance the planner will use
            */
   public double maximum_xy_wiggle_distance_ = -1.0;
   /**
            * When wiggling a candidate footstep into a planar region, this is the maximum yaw
            * distance the planner will use
            */
   public double maximum_yaw_wiggle_ = -1.0;
   /**
            * When snapping a candidate footstep to a planar region, its possible that another planar region
            * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
            * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
            * otherwise it will.
            */
   public double maximum_z_penetration_on_valley_regions_ = -1.0;
   /**
            * Maximum step width the planner will consider for candidate steps.
            * 
            * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
            * where the parent is the last footstep taken on the other foot.
            * 
            * If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
            */
   public double maximum_step_width_ = -1.0;
   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double cliff_height_to_avoid_ = -1.0;
   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
            * until it is minimumDistanceFromCliffBottoms away from it.
            * 
            * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
            * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
            * generator is capable of swinging over.
            */
   public double minimum_distance_from_cliff_bottoms_ = -1.0;
   /**
            * When the planner is done planning and cannot find a path to the goal, this flag indicates whether the
            * planner should return the best plan that it found. If this value is false, the planner will return
            * a {@link FootstepPlan} of type {@link FootstepPlanningResult#NO_PATH_EXISTS}. Otherwise it will return
            * "best" is determined by the planner.
            */
   public boolean return_best_effort_plan_;
   /**
            * When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
            * contains at least this many footsteps.
            */
   public long minimum_steps_for_best_effort_plan_;
   /**
            * Some node checkers will check if the body of the robot will move through a higher planar region
            * (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
            * ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
            * This should be set to be slightly above cinder block height (20.3cm) for Atlas.
            */
   public double body_ground_clearance_ = -1.0;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box height.
            */
   public double body_box_height_ = -1.0;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box depth.
            */
   public double body_box_depth_ = -1.0;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the box width.
            */
   public double body_box_width_ = -1.0;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double body_box_base_x_ = -1.0;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double body_box_base_y_ = -1.0;
   /**
            * Some node checkers will check if a bounding box that describes the body of the robot will move
            * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
            * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
            */
   public double body_box_base_z_ = -1.0;
   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public double min_x_clearance_from_stance_ = -1.0;
   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public double min_y_clearance_from_stance_ = -1.0;
   /**
            * Radius around the goal inside which the planner should start to turn to match the goal's orientation
            */
   public double final_turn_proximity_ = -1.0;
   /**
            * Radius around the goal inside which the body path heuristic planner should start to turn to match the goal's orientation
            */
   public double final_turn_body_path_proximity_ = -1.0;
   /**
            * Defines a percentage of the radius around the final turn proximity in which the blending from the desired heading to the
            * final orientation should occur. That is, at 1 + {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
            * the desired orientation is the desired heading, and at 1 - {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
            * the desired orientation is the final orientation.
            */
   public double final_turn_proximity_blend_factor_ = -1.0;
   /**
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public double yaw_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public double pitch_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public double roll_weight_ = -1.0;
   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public double forward_weight_ = -1.0;
   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public double lateral_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public double step_up_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public double step_down_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public double long_step_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
            */
   public double foothold_area_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public double cost_per_step_ = -1.0;
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
            */
   public boolean use_quadratic_distance_cost_;
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
            */
   public boolean use_quadratic_height_cost_;
   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public double a_star_heuristics_weight_ = -1.0;
   /**
            * Gets the weight for the heuristics in the Visibility graph with A star planner.
            */
   public double vis_graph_with_a_star_heuristics_weight_ = -1.0;
   /**
            * Gets the weight for the heuristics in the Depth First planner.
            */
   public double depth_first_heuristics_weight_ = -1.0;
   /**
            * Gets the weight for the heuristics in the Body path based planner.
            */
   public double body_path_based_heuristics_weight_ = -1.0;
   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public double maximum_2d_distance_from_bounding_box_to_penalize_ = -1.0;
   /**
            * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
            * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
            * {@code c * (1 - d / d_max)}, where d_max is this value.
            */
   public double bounding_box_cost_ = -1.0;
   public double body_path_violation_weight_ = -1.0;
   public double distance_from_path_tolerance_ = -1.0;
   public double delta_yaw_from_reference_tolerance_ = -1.0;

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

      perform_heuristic_search_policies_ = other.perform_heuristic_search_policies_;

      ideal_footstep_width_ = other.ideal_footstep_width_;

      ideal_footstep_length_ = other.ideal_footstep_length_;

      wiggle_inside_delta_ = other.wiggle_inside_delta_;

      maximum_step_reach_ = other.maximum_step_reach_;

      maximum_step_yaw_ = other.maximum_step_yaw_;

      minimum_step_width_ = other.minimum_step_width_;

      minimum_step_length_ = other.minimum_step_length_;

      minimum_step_yaw_ = other.minimum_step_yaw_;

      maximum_step_reach_when_stepping_up_ = other.maximum_step_reach_when_stepping_up_;

      maximum_step_z_when_stepping_up_ = other.maximum_step_z_when_stepping_up_;

      maximum_step_x_when_forward_and_down_ = other.maximum_step_x_when_forward_and_down_;

      maximum_step_z_when_forward_and_down_ = other.maximum_step_z_when_forward_and_down_;

      maximum_step_z_ = other.maximum_step_z_;

      minimum_foothold_percent_ = other.minimum_foothold_percent_;

      minimum_surface_incline_radians_ = other.minimum_surface_incline_radians_;

      wiggle_into_convex_hull_of_planar_regions_ = other.wiggle_into_convex_hull_of_planar_regions_;

      reject_if_cannot_fully_wiggle_inside_ = other.reject_if_cannot_fully_wiggle_inside_;

      maximum_xy_wiggle_distance_ = other.maximum_xy_wiggle_distance_;

      maximum_yaw_wiggle_ = other.maximum_yaw_wiggle_;

      maximum_z_penetration_on_valley_regions_ = other.maximum_z_penetration_on_valley_regions_;

      maximum_step_width_ = other.maximum_step_width_;

      cliff_height_to_avoid_ = other.cliff_height_to_avoid_;

      minimum_distance_from_cliff_bottoms_ = other.minimum_distance_from_cliff_bottoms_;

      return_best_effort_plan_ = other.return_best_effort_plan_;

      minimum_steps_for_best_effort_plan_ = other.minimum_steps_for_best_effort_plan_;

      body_ground_clearance_ = other.body_ground_clearance_;

      body_box_height_ = other.body_box_height_;

      body_box_depth_ = other.body_box_depth_;

      body_box_width_ = other.body_box_width_;

      body_box_base_x_ = other.body_box_base_x_;

      body_box_base_y_ = other.body_box_base_y_;

      body_box_base_z_ = other.body_box_base_z_;

      min_x_clearance_from_stance_ = other.min_x_clearance_from_stance_;

      min_y_clearance_from_stance_ = other.min_y_clearance_from_stance_;

      final_turn_proximity_ = other.final_turn_proximity_;

      final_turn_body_path_proximity_ = other.final_turn_body_path_proximity_;

      final_turn_proximity_blend_factor_ = other.final_turn_proximity_blend_factor_;

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

      use_quadratic_distance_cost_ = other.use_quadratic_distance_cost_;

      use_quadratic_height_cost_ = other.use_quadratic_height_cost_;

      a_star_heuristics_weight_ = other.a_star_heuristics_weight_;

      vis_graph_with_a_star_heuristics_weight_ = other.vis_graph_with_a_star_heuristics_weight_;

      depth_first_heuristics_weight_ = other.depth_first_heuristics_weight_;

      body_path_based_heuristics_weight_ = other.body_path_based_heuristics_weight_;

      maximum_2d_distance_from_bounding_box_to_penalize_ = other.maximum_2d_distance_from_bounding_box_to_penalize_;

      bounding_box_cost_ = other.bounding_box_cost_;

      body_path_violation_weight_ = other.body_path_violation_weight_;

      distance_from_path_tolerance_ = other.distance_from_path_tolerance_;

      delta_yaw_from_reference_tolerance_ = other.delta_yaw_from_reference_tolerance_;

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
            * Sets whether or not to perform the defined heuristic search policies.
            */
   public void setPerformHeuristicSearchPolicies(boolean perform_heuristic_search_policies)
   {
      perform_heuristic_search_policies_ = perform_heuristic_search_policies;
   }
   /**
            * Sets whether or not to perform the defined heuristic search policies.
            */
   public boolean getPerformHeuristicSearchPolicies()
   {
      return perform_heuristic_search_policies_;
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
            * If the planner in use utilized footstep wiggling (see {@link PolygonWiggler}) to move footholds onto planer
            * regions this parameter will be used. It specifies the minimum distance between the foot polygon and the
            * edge of the planar region polygon that the footstep is moved into. This value can be negative. That corresponds
            * to allowing footsteps that partially intersect planar regions.
            * 
            * If this value is too high, the planner will not put footsteps on small planar regions. At zero, the planner might
            * choose a footstep with an edge along a planar region. This value should roughly be set to the sum of two values:
            * The smallest acceptable distance to the edge of a cliff
            * The maximum error between desired and actual foot placement
            */
   public void setWiggleInsideDelta(double wiggle_inside_delta)
   {
      wiggle_inside_delta_ = wiggle_inside_delta;
   }
   /**
            * If the planner in use utilized footstep wiggling (see {@link PolygonWiggler}) to move footholds onto planer
            * regions this parameter will be used. It specifies the minimum distance between the foot polygon and the
            * edge of the planar region polygon that the footstep is moved into. This value can be negative. That corresponds
            * to allowing footsteps that partially intersect planar regions.
            * 
            * If this value is too high, the planner will not put footsteps on small planar regions. At zero, the planner might
            * choose a footstep with an edge along a planar region. This value should roughly be set to the sum of two values:
            * The smallest acceptable distance to the edge of a cliff
            * The maximum error between desired and actual foot placement
            */
   public double getWiggleInsideDelta()
   {
      return wiggle_inside_delta_;
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
            * Large steps forward and down are rejected by the planner if two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
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
            * Large steps forward and down are rejected by the planner if two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
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
            * Maximum step height when stepping forward and down.
            * 
            * Large steps forward and down are rejected by the planner if two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * The z-position of the value of the footstep is less than -maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
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
            * Large steps forward and down are rejected by the planner if two criteria are met:
            * The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame
            * The z-position of the value of the footstep is less than -maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame
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
            * There are two methods of wiggling a polygon into a planar region:
            * Wiggle the polygon into the planar region itself, which isn't necessarily convex
            * Wiggle the polygon into the convex hull of the planar region
            * The first method is not implemented completely. Instead it will wiggle into the sub polygon of the planar region that
            * has the biggest overlap with the foothold.
            * 
            * If this parameter is set to true (recommended), the second wiggle method will be used.
            */
   public void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggle_into_convex_hull_of_planar_regions)
   {
      wiggle_into_convex_hull_of_planar_regions_ = wiggle_into_convex_hull_of_planar_regions;
   }
   /**
            * There are two methods of wiggling a polygon into a planar region:
            * Wiggle the polygon into the planar region itself, which isn't necessarily convex
            * Wiggle the polygon into the convex hull of the planar region
            * The first method is not implemented completely. Instead it will wiggle into the sub polygon of the planar region that
            * has the biggest overlap with the foothold.
            * 
            * If this parameter is set to true (recommended), the second wiggle method will be used.
            */
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return wiggle_into_convex_hull_of_planar_regions_;
   }

   /**
            * If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
            * This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
            * {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
            * candidate footstep will be rejected if this method returns {@code true}.
            */
   public void setRejectIfCannotFullyWiggleInside(boolean reject_if_cannot_fully_wiggle_inside)
   {
      reject_if_cannot_fully_wiggle_inside_ = reject_if_cannot_fully_wiggle_inside;
   }
   /**
            * If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
            * This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
            * {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
            * candidate footstep will be rejected if this method returns {@code true}.
            */
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return reject_if_cannot_fully_wiggle_inside_;
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
   public void setCliffHeightToAvoid(double cliff_height_to_avoid)
   {
      cliff_height_to_avoid_ = cliff_height_to_avoid;
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
   public double getCliffHeightToAvoid()
   {
      return cliff_height_to_avoid_;
   }

   /**
            * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
            * nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
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
            * nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
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
            * When the planner is done planning and cannot find a path to the goal, this flag indicates whether the
            * planner should return the best plan that it found. If this value is false, the planner will return
            * a {@link FootstepPlan} of type {@link FootstepPlanningResult#NO_PATH_EXISTS}. Otherwise it will return
            * "best" is determined by the planner.
            */
   public void setReturnBestEffortPlan(boolean return_best_effort_plan)
   {
      return_best_effort_plan_ = return_best_effort_plan;
   }
   /**
            * When the planner is done planning and cannot find a path to the goal, this flag indicates whether the
            * planner should return the best plan that it found. If this value is false, the planner will return
            * a {@link FootstepPlan} of type {@link FootstepPlanningResult#NO_PATH_EXISTS}. Otherwise it will return
            * "best" is determined by the planner.
            */
   public boolean getReturnBestEffortPlan()
   {
      return return_best_effort_plan_;
   }

   /**
            * When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
            * contains at least this many footsteps.
            */
   public void setMinimumStepsForBestEffortPlan(long minimum_steps_for_best_effort_plan)
   {
      minimum_steps_for_best_effort_plan_ = minimum_steps_for_best_effort_plan;
   }
   /**
            * When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
            * contains at least this many footsteps.
            */
   public long getMinimumStepsForBestEffortPlan()
   {
      return minimum_steps_for_best_effort_plan_;
   }

   /**
            * Some node checkers will check if the body of the robot will move through a higher planar region
            * (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
            * ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
            * This should be set to be slightly above cinder block height (20.3cm) for Atlas.
            */
   public void setBodyGroundClearance(double body_ground_clearance)
   {
      body_ground_clearance_ = body_ground_clearance;
   }
   /**
            * Some node checkers will check if the body of the robot will move through a higher planar region
            * (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
            * ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
            * This should be set to be slightly above cinder block height (20.3cm) for Atlas.
            */
   public double getBodyGroundClearance()
   {
      return body_ground_clearance_;
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
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public void setMinXClearanceFromStance(double min_x_clearance_from_stance)
   {
      min_x_clearance_from_stance_ = min_x_clearance_from_stance;
   }
   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public double getMinXClearanceFromStance()
   {
      return min_x_clearance_from_stance_;
   }

   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public void setMinYClearanceFromStance(double min_y_clearance_from_stance)
   {
      min_y_clearance_from_stance_ = min_y_clearance_from_stance;
   }
   /**
            * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
            * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
            * this parameter.
            */
   public double getMinYClearanceFromStance()
   {
      return min_y_clearance_from_stance_;
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
            * Radius around the goal inside which the body path heuristic planner should start to turn to match the goal's orientation
            */
   public void setFinalTurnBodyPathProximity(double final_turn_body_path_proximity)
   {
      final_turn_body_path_proximity_ = final_turn_body_path_proximity;
   }
   /**
            * Radius around the goal inside which the body path heuristic planner should start to turn to match the goal's orientation
            */
   public double getFinalTurnBodyPathProximity()
   {
      return final_turn_body_path_proximity_;
   }

   /**
            * Defines a percentage of the radius around the final turn proximity in which the blending from the desired heading to the
            * final orientation should occur. That is, at 1 + {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
            * the desired orientation is the desired heading, and at 1 - {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
            * the desired orientation is the final orientation.
            */
   public void setFinalTurnProximityBlendFactor(double final_turn_proximity_blend_factor)
   {
      final_turn_proximity_blend_factor_ = final_turn_proximity_blend_factor;
   }
   /**
            * Defines a percentage of the radius around the final turn proximity in which the blending from the desired heading to the
            * final orientation should occur. That is, at 1 + {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
            * the desired orientation is the desired heading, and at 1 - {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()},
            * the desired orientation is the final orientation.
            */
   public double getFinalTurnProximityBlendFactor()
   {
      return final_turn_proximity_blend_factor_;
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
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
            */
   public void setUseQuadraticDistanceCost(boolean use_quadratic_distance_cost)
   {
      use_quadratic_distance_cost_ = use_quadratic_distance_cost;
   }
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
            */
   public boolean getUseQuadraticDistanceCost()
   {
      return use_quadratic_distance_cost_;
   }

   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
            */
   public void setUseQuadraticHeightCost(boolean use_quadratic_height_cost)
   {
      use_quadratic_height_cost_ = use_quadratic_height_cost;
   }
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
            */
   public boolean getUseQuadraticHeightCost()
   {
      return use_quadratic_height_cost_;
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
            * Gets the weight for the heuristics in the Visibility graph with A star planner.
            */
   public void setVisGraphWithAStarHeuristicsWeight(double vis_graph_with_a_star_heuristics_weight)
   {
      vis_graph_with_a_star_heuristics_weight_ = vis_graph_with_a_star_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the Visibility graph with A star planner.
            */
   public double getVisGraphWithAStarHeuristicsWeight()
   {
      return vis_graph_with_a_star_heuristics_weight_;
   }

   /**
            * Gets the weight for the heuristics in the Depth First planner.
            */
   public void setDepthFirstHeuristicsWeight(double depth_first_heuristics_weight)
   {
      depth_first_heuristics_weight_ = depth_first_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the Depth First planner.
            */
   public double getDepthFirstHeuristicsWeight()
   {
      return depth_first_heuristics_weight_;
   }

   /**
            * Gets the weight for the heuristics in the Body path based planner.
            */
   public void setBodyPathBasedHeuristicsWeight(double body_path_based_heuristics_weight)
   {
      body_path_based_heuristics_weight_ = body_path_based_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the Body path based planner.
            */
   public double getBodyPathBasedHeuristicsWeight()
   {
      return body_path_based_heuristics_weight_;
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
            * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
            * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
            * {@code c * (1 - d / d_max)}, where d_max is this value.
            */
   public void setBoundingBoxCost(double bounding_box_cost)
   {
      bounding_box_cost_ = bounding_box_cost;
   }
   /**
            * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
            * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
            * {@code c * (1 - d / d_max)}, where d_max is this value.
            */
   public double getBoundingBoxCost()
   {
      return bounding_box_cost_;
   }

   public void setBodyPathViolationWeight(double body_path_violation_weight)
   {
      body_path_violation_weight_ = body_path_violation_weight;
   }
   public double getBodyPathViolationWeight()
   {
      return body_path_violation_weight_;
   }

   public void setDistanceFromPathTolerance(double distance_from_path_tolerance)
   {
      distance_from_path_tolerance_ = distance_from_path_tolerance;
   }
   public double getDistanceFromPathTolerance()
   {
      return distance_from_path_tolerance_;
   }

   public void setDeltaYawFromReferenceTolerance(double delta_yaw_from_reference_tolerance)
   {
      delta_yaw_from_reference_tolerance_ = delta_yaw_from_reference_tolerance;
   }
   public double getDeltaYawFromReferenceTolerance()
   {
      return delta_yaw_from_reference_tolerance_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.perform_heuristic_search_policies_, other.perform_heuristic_search_policies_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_footstep_width_, other.ideal_footstep_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ideal_footstep_length_, other.ideal_footstep_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wiggle_inside_delta_, other.wiggle_inside_delta_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_, other.maximum_step_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_yaw_, other.maximum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_width_, other.minimum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_length_, other.minimum_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_yaw_, other.minimum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_when_stepping_up_, other.maximum_step_reach_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_when_stepping_up_, other.maximum_step_z_when_stepping_up_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_x_when_forward_and_down_, other.maximum_step_x_when_forward_and_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_when_forward_and_down_, other.maximum_step_z_when_forward_and_down_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_z_, other.maximum_step_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_foothold_percent_, other.minimum_foothold_percent_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_surface_incline_radians_, other.minimum_surface_incline_radians_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.wiggle_into_convex_hull_of_planar_regions_, other.wiggle_into_convex_hull_of_planar_regions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.reject_if_cannot_fully_wiggle_inside_, other.reject_if_cannot_fully_wiggle_inside_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_xy_wiggle_distance_, other.maximum_xy_wiggle_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_yaw_wiggle_, other.maximum_yaw_wiggle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_z_penetration_on_valley_regions_, other.maximum_z_penetration_on_valley_regions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_, other.maximum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cliff_height_to_avoid_, other.cliff_height_to_avoid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_distance_from_cliff_bottoms_, other.minimum_distance_from_cliff_bottoms_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.return_best_effort_plan_, other.return_best_effort_plan_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_steps_for_best_effort_plan_, other.minimum_steps_for_best_effort_plan_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_ground_clearance_, other.body_ground_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_height_, other.body_box_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_depth_, other.body_box_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_width_, other.body_box_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_base_x_, other.body_box_base_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_base_y_, other.body_box_base_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_box_base_z_, other.body_box_base_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_x_clearance_from_stance_, other.min_x_clearance_from_stance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_y_clearance_from_stance_, other.min_y_clearance_from_stance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_turn_proximity_, other.final_turn_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_turn_body_path_proximity_, other.final_turn_body_path_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_turn_proximity_blend_factor_, other.final_turn_proximity_blend_factor_, epsilon)) return false;

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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_quadratic_distance_cost_, other.use_quadratic_distance_cost_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_quadratic_height_cost_, other.use_quadratic_height_cost_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.a_star_heuristics_weight_, other.a_star_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vis_graph_with_a_star_heuristics_weight_, other.vis_graph_with_a_star_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.depth_first_heuristics_weight_, other.depth_first_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_path_based_heuristics_weight_, other.body_path_based_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_2d_distance_from_bounding_box_to_penalize_, other.maximum_2d_distance_from_bounding_box_to_penalize_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bounding_box_cost_, other.bounding_box_cost_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_path_violation_weight_, other.body_path_violation_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_from_path_tolerance_, other.distance_from_path_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.delta_yaw_from_reference_tolerance_, other.delta_yaw_from_reference_tolerance_, epsilon)) return false;


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

      if(this.perform_heuristic_search_policies_ != otherMyClass.perform_heuristic_search_policies_) return false;

      if(this.ideal_footstep_width_ != otherMyClass.ideal_footstep_width_) return false;

      if(this.ideal_footstep_length_ != otherMyClass.ideal_footstep_length_) return false;

      if(this.wiggle_inside_delta_ != otherMyClass.wiggle_inside_delta_) return false;

      if(this.maximum_step_reach_ != otherMyClass.maximum_step_reach_) return false;

      if(this.maximum_step_yaw_ != otherMyClass.maximum_step_yaw_) return false;

      if(this.minimum_step_width_ != otherMyClass.minimum_step_width_) return false;

      if(this.minimum_step_length_ != otherMyClass.minimum_step_length_) return false;

      if(this.minimum_step_yaw_ != otherMyClass.minimum_step_yaw_) return false;

      if(this.maximum_step_reach_when_stepping_up_ != otherMyClass.maximum_step_reach_when_stepping_up_) return false;

      if(this.maximum_step_z_when_stepping_up_ != otherMyClass.maximum_step_z_when_stepping_up_) return false;

      if(this.maximum_step_x_when_forward_and_down_ != otherMyClass.maximum_step_x_when_forward_and_down_) return false;

      if(this.maximum_step_z_when_forward_and_down_ != otherMyClass.maximum_step_z_when_forward_and_down_) return false;

      if(this.maximum_step_z_ != otherMyClass.maximum_step_z_) return false;

      if(this.minimum_foothold_percent_ != otherMyClass.minimum_foothold_percent_) return false;

      if(this.minimum_surface_incline_radians_ != otherMyClass.minimum_surface_incline_radians_) return false;

      if(this.wiggle_into_convex_hull_of_planar_regions_ != otherMyClass.wiggle_into_convex_hull_of_planar_regions_) return false;

      if(this.reject_if_cannot_fully_wiggle_inside_ != otherMyClass.reject_if_cannot_fully_wiggle_inside_) return false;

      if(this.maximum_xy_wiggle_distance_ != otherMyClass.maximum_xy_wiggle_distance_) return false;

      if(this.maximum_yaw_wiggle_ != otherMyClass.maximum_yaw_wiggle_) return false;

      if(this.maximum_z_penetration_on_valley_regions_ != otherMyClass.maximum_z_penetration_on_valley_regions_) return false;

      if(this.maximum_step_width_ != otherMyClass.maximum_step_width_) return false;

      if(this.cliff_height_to_avoid_ != otherMyClass.cliff_height_to_avoid_) return false;

      if(this.minimum_distance_from_cliff_bottoms_ != otherMyClass.minimum_distance_from_cliff_bottoms_) return false;

      if(this.return_best_effort_plan_ != otherMyClass.return_best_effort_plan_) return false;

      if(this.minimum_steps_for_best_effort_plan_ != otherMyClass.minimum_steps_for_best_effort_plan_) return false;

      if(this.body_ground_clearance_ != otherMyClass.body_ground_clearance_) return false;

      if(this.body_box_height_ != otherMyClass.body_box_height_) return false;

      if(this.body_box_depth_ != otherMyClass.body_box_depth_) return false;

      if(this.body_box_width_ != otherMyClass.body_box_width_) return false;

      if(this.body_box_base_x_ != otherMyClass.body_box_base_x_) return false;

      if(this.body_box_base_y_ != otherMyClass.body_box_base_y_) return false;

      if(this.body_box_base_z_ != otherMyClass.body_box_base_z_) return false;

      if(this.min_x_clearance_from_stance_ != otherMyClass.min_x_clearance_from_stance_) return false;

      if(this.min_y_clearance_from_stance_ != otherMyClass.min_y_clearance_from_stance_) return false;

      if(this.final_turn_proximity_ != otherMyClass.final_turn_proximity_) return false;

      if(this.final_turn_body_path_proximity_ != otherMyClass.final_turn_body_path_proximity_) return false;

      if(this.final_turn_proximity_blend_factor_ != otherMyClass.final_turn_proximity_blend_factor_) return false;

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

      if(this.use_quadratic_distance_cost_ != otherMyClass.use_quadratic_distance_cost_) return false;

      if(this.use_quadratic_height_cost_ != otherMyClass.use_quadratic_height_cost_) return false;

      if(this.a_star_heuristics_weight_ != otherMyClass.a_star_heuristics_weight_) return false;

      if(this.vis_graph_with_a_star_heuristics_weight_ != otherMyClass.vis_graph_with_a_star_heuristics_weight_) return false;

      if(this.depth_first_heuristics_weight_ != otherMyClass.depth_first_heuristics_weight_) return false;

      if(this.body_path_based_heuristics_weight_ != otherMyClass.body_path_based_heuristics_weight_) return false;

      if(this.maximum_2d_distance_from_bounding_box_to_penalize_ != otherMyClass.maximum_2d_distance_from_bounding_box_to_penalize_) return false;

      if(this.bounding_box_cost_ != otherMyClass.bounding_box_cost_) return false;

      if(this.body_path_violation_weight_ != otherMyClass.body_path_violation_weight_) return false;

      if(this.distance_from_path_tolerance_ != otherMyClass.distance_from_path_tolerance_) return false;

      if(this.delta_yaw_from_reference_tolerance_ != otherMyClass.delta_yaw_from_reference_tolerance_) return false;


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
      builder.append("perform_heuristic_search_policies=");
      builder.append(this.perform_heuristic_search_policies_);      builder.append(", ");
      builder.append("ideal_footstep_width=");
      builder.append(this.ideal_footstep_width_);      builder.append(", ");
      builder.append("ideal_footstep_length=");
      builder.append(this.ideal_footstep_length_);      builder.append(", ");
      builder.append("wiggle_inside_delta=");
      builder.append(this.wiggle_inside_delta_);      builder.append(", ");
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
      builder.append("maximum_step_z_when_stepping_up=");
      builder.append(this.maximum_step_z_when_stepping_up_);      builder.append(", ");
      builder.append("maximum_step_x_when_forward_and_down=");
      builder.append(this.maximum_step_x_when_forward_and_down_);      builder.append(", ");
      builder.append("maximum_step_z_when_forward_and_down=");
      builder.append(this.maximum_step_z_when_forward_and_down_);      builder.append(", ");
      builder.append("maximum_step_z=");
      builder.append(this.maximum_step_z_);      builder.append(", ");
      builder.append("minimum_foothold_percent=");
      builder.append(this.minimum_foothold_percent_);      builder.append(", ");
      builder.append("minimum_surface_incline_radians=");
      builder.append(this.minimum_surface_incline_radians_);      builder.append(", ");
      builder.append("wiggle_into_convex_hull_of_planar_regions=");
      builder.append(this.wiggle_into_convex_hull_of_planar_regions_);      builder.append(", ");
      builder.append("reject_if_cannot_fully_wiggle_inside=");
      builder.append(this.reject_if_cannot_fully_wiggle_inside_);      builder.append(", ");
      builder.append("maximum_xy_wiggle_distance=");
      builder.append(this.maximum_xy_wiggle_distance_);      builder.append(", ");
      builder.append("maximum_yaw_wiggle=");
      builder.append(this.maximum_yaw_wiggle_);      builder.append(", ");
      builder.append("maximum_z_penetration_on_valley_regions=");
      builder.append(this.maximum_z_penetration_on_valley_regions_);      builder.append(", ");
      builder.append("maximum_step_width=");
      builder.append(this.maximum_step_width_);      builder.append(", ");
      builder.append("cliff_height_to_avoid=");
      builder.append(this.cliff_height_to_avoid_);      builder.append(", ");
      builder.append("minimum_distance_from_cliff_bottoms=");
      builder.append(this.minimum_distance_from_cliff_bottoms_);      builder.append(", ");
      builder.append("return_best_effort_plan=");
      builder.append(this.return_best_effort_plan_);      builder.append(", ");
      builder.append("minimum_steps_for_best_effort_plan=");
      builder.append(this.minimum_steps_for_best_effort_plan_);      builder.append(", ");
      builder.append("body_ground_clearance=");
      builder.append(this.body_ground_clearance_);      builder.append(", ");
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
      builder.append("min_x_clearance_from_stance=");
      builder.append(this.min_x_clearance_from_stance_);      builder.append(", ");
      builder.append("min_y_clearance_from_stance=");
      builder.append(this.min_y_clearance_from_stance_);      builder.append(", ");
      builder.append("final_turn_proximity=");
      builder.append(this.final_turn_proximity_);      builder.append(", ");
      builder.append("final_turn_body_path_proximity=");
      builder.append(this.final_turn_body_path_proximity_);      builder.append(", ");
      builder.append("final_turn_proximity_blend_factor=");
      builder.append(this.final_turn_proximity_blend_factor_);      builder.append(", ");
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
      builder.append("use_quadratic_distance_cost=");
      builder.append(this.use_quadratic_distance_cost_);      builder.append(", ");
      builder.append("use_quadratic_height_cost=");
      builder.append(this.use_quadratic_height_cost_);      builder.append(", ");
      builder.append("a_star_heuristics_weight=");
      builder.append(this.a_star_heuristics_weight_);      builder.append(", ");
      builder.append("vis_graph_with_a_star_heuristics_weight=");
      builder.append(this.vis_graph_with_a_star_heuristics_weight_);      builder.append(", ");
      builder.append("depth_first_heuristics_weight=");
      builder.append(this.depth_first_heuristics_weight_);      builder.append(", ");
      builder.append("body_path_based_heuristics_weight=");
      builder.append(this.body_path_based_heuristics_weight_);      builder.append(", ");
      builder.append("maximum_2d_distance_from_bounding_box_to_penalize=");
      builder.append(this.maximum_2d_distance_from_bounding_box_to_penalize_);      builder.append(", ");
      builder.append("bounding_box_cost=");
      builder.append(this.bounding_box_cost_);      builder.append(", ");
      builder.append("body_path_violation_weight=");
      builder.append(this.body_path_violation_weight_);      builder.append(", ");
      builder.append("distance_from_path_tolerance=");
      builder.append(this.distance_from_path_tolerance_);      builder.append(", ");
      builder.append("delta_yaw_from_reference_tolerance=");
      builder.append(this.delta_yaw_from_reference_tolerance_);
      builder.append("}");
      return builder.toString();
   }
}
