package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       * This message defines the parameters for the A* body path planner.
       */
public class AStarBodyPathPlannerParametersPacket extends Packet<AStarBodyPathPlannerParametersPacket> implements Settable<AStarBodyPathPlannerParametersPacket>, EpsilonComparable<AStarBodyPathPlannerParametersPacket>
{
   /**
            * Unique ID used to identify this message.
            */
   public long sequence_id_;
   /**
            * Whether or not the planner checks for collisions.
            */
   public boolean check_for_collisions_;
   /**
            * Whether the planner computes surface normals. If false, traversibility and roll are also not computed.
            */
   public boolean compute_surface_normal_cost_;
   /**
            * Whether the planner computes and checks traversibility.
            */
   public boolean compute_traversibility_;
   /**
            * Whether the body path plan is post-processed with the smoother.
            */
   public boolean perform_smoothing_;
   /**
            * This is the weight assigned to roll in the search space. Increasing this value will decrease the likelihood of the path moving sideways across slopes.
            */
   public double roll_cost_weight_;
   /**
            * The angle in degrees of the deadband applied to the computed roll. Increasing this value decreases the effect small rolls have on the path.
            */
   public double roll_cost_deadband_;
   /**
            * When the roll is below this angle, the cost is linearly discounted to zero. Increasing this value decreases the effect roll has on the path.
            */
   public double max_penalized_roll_angle_;
   /**
            * The node height of a vertex is determined as the average height of all cells within this radius.
            */
   public double snap_radius_;
   /**
            * When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
            */
   public double min_snap_height_threshold_;
   /**
            * Weight assigned to minimizing the incline the path takes.
            */
   public double incline_cost_weight_;
   /**
            * Deadband applied to the incline in the search.
            */
   public double incline_cost_deadband_;
   /**
            * The maximum incline, in degrees, that is allowed for the body path planner to traverse.
            */
   public double max_incline_;
   /**
            * Width of the collision box used for checking collisions with the environment.
            */
   public double collision_box_size_y_;
   /**
            * Depth of the collision box used for checking collisions with the environment.
            */
   public double collision_box_size_x_;
   /**
            * Intersection height below which collisions are ignored.
            */
   public double collision_box_ground_clearance_;
   /**
            * Weight placed on maximizing traversibility in the plan.
            */
   public double traversibility_weight_;
   /**
            * Weight placed on the traversibility at the start node when computing the overall traversibility score.
            */
   public double traversibility_stance_weight_;
   /**
            * Weight placed on the traversibility at the end node when computing the overall traversibility score.
            */
   public double traversibility_step_weight_;
   /**
            * Minimum score on the start node traversibility to say an edge is traversible.
            */
   public double min_traversibility_score_;
   /**
            * Minimum angle in degrees to penalize in the traversibility score.
            */
   public double min_normal_angle_to_penalize_for_traversibility_;
   /**
            * Maximum angle in degrees to penalize in the traversibility score.
            */
   public double max_normal_angle_to_penalize_for_traversibility_;
   /**
            * Weight to place on the surface normals when computing the traversibility score.
            */
   public double traversibility_incline_weight_;
   /**
            * Box width of cells to include when performing the traversibility calculation.
            */
   public double traversibility_search_width_;
   /**
            * Minimum number of occupied neighbor cells to say a vertex is traversible.
            */
   public int min_occupied_neighbors_for_traversibility_;
   /**
            * Half the typical stance width of the robot, used to compute the traversibility.
            */
   public double half_stance_width_;
   /**
            * Width of the height window of cells to include in the traversibility calculation.
            */
   public double traversibility_height_window_width_;
   /**
            * Deadband applied to the height distance of cells in the traversibility calculation.
            */
   public double traversibility_height_window_deadband_;
   /**
            * Distance to the ground plane estimate that is needed to say the two nodes are in the ground plane.
            */
   public double height_proximity_for_saying_walking_on_ground_;
   /**
            * Minimum discount applied to non-ground cells when computing the traversibility on the ground plane.
            */
   public double traversibility_non_ground_discount_when_walking_on_ground_;
   /**
            * Weight placed on the gradient for avoiding collisions in the smoother.
            */
   public double smoother_collision_weight_;
   /**
            * Weight placed on the gradient for minimizing the angle between successive segments of the body path.
            */
   public double smoother_smoothness_weight_;
   /**
            * Discount applied to the smoothness gradients of turn points.
            */
   public double smoother_turn_point_smoothness_discount_;
   /**
            * Minimum curvature in degrees to penalize with a gradient.
            */
   public double smoother_min_curvature_to_penalize_;
   /**
            * Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
            */
   public double smoother_equal_spacing_weight_;
   /**
            * Weight placed on the gradient for minimizing the roll cost of the body path plan.
            */
   public double smoother_roll_weight_;
   /**
            * Weight placed on a gradient that drives the waypoint towards the initial value.
            */
   public double smoother_displacement_weight_;
   /**
            * Weight placed on the gradient for maximizing the traversibility of the body path plan.
            */
   public double smoother_traversibility_weight_;
   /**
            * Weight placed on the gradient pushing the body path plan towards cells in the ground plane.
            */
   public double smoother_ground_plane_weight_;
   /**
            * Traversibility threshold resulting in a zero gradient for traversibility.
            */
   public double smoother_minimum_traversibility_to_search_for_;
   /**
            * Traversibility threshold above which the traversibility gradient begins to carry less weight.
            */
   public double smoother_traversibility_threshold_for_no_discount_;
   /**
            * Gain applied to the smoother gradient for iterative modifications.
            */
   public double smoother_hill_climb_gain_;
   /**
            * Minimum gradient vector magnitude to terminate the smoother iterations.
            */
   public double smoother_gradient_threshold_to_terminate_;

   public AStarBodyPathPlannerParametersPacket()
   {
   }

   public AStarBodyPathPlannerParametersPacket(AStarBodyPathPlannerParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(AStarBodyPathPlannerParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      check_for_collisions_ = other.check_for_collisions_;

      compute_surface_normal_cost_ = other.compute_surface_normal_cost_;

      compute_traversibility_ = other.compute_traversibility_;

      perform_smoothing_ = other.perform_smoothing_;

      roll_cost_weight_ = other.roll_cost_weight_;

      roll_cost_deadband_ = other.roll_cost_deadband_;

      max_penalized_roll_angle_ = other.max_penalized_roll_angle_;

      snap_radius_ = other.snap_radius_;

      min_snap_height_threshold_ = other.min_snap_height_threshold_;

      incline_cost_weight_ = other.incline_cost_weight_;

      incline_cost_deadband_ = other.incline_cost_deadband_;

      max_incline_ = other.max_incline_;

      collision_box_size_y_ = other.collision_box_size_y_;

      collision_box_size_x_ = other.collision_box_size_x_;

      collision_box_ground_clearance_ = other.collision_box_ground_clearance_;

      traversibility_weight_ = other.traversibility_weight_;

      traversibility_stance_weight_ = other.traversibility_stance_weight_;

      traversibility_step_weight_ = other.traversibility_step_weight_;

      min_traversibility_score_ = other.min_traversibility_score_;

      min_normal_angle_to_penalize_for_traversibility_ = other.min_normal_angle_to_penalize_for_traversibility_;

      max_normal_angle_to_penalize_for_traversibility_ = other.max_normal_angle_to_penalize_for_traversibility_;

      traversibility_incline_weight_ = other.traversibility_incline_weight_;

      traversibility_search_width_ = other.traversibility_search_width_;

      min_occupied_neighbors_for_traversibility_ = other.min_occupied_neighbors_for_traversibility_;

      half_stance_width_ = other.half_stance_width_;

      traversibility_height_window_width_ = other.traversibility_height_window_width_;

      traversibility_height_window_deadband_ = other.traversibility_height_window_deadband_;

      height_proximity_for_saying_walking_on_ground_ = other.height_proximity_for_saying_walking_on_ground_;

      traversibility_non_ground_discount_when_walking_on_ground_ = other.traversibility_non_ground_discount_when_walking_on_ground_;

      smoother_collision_weight_ = other.smoother_collision_weight_;

      smoother_smoothness_weight_ = other.smoother_smoothness_weight_;

      smoother_turn_point_smoothness_discount_ = other.smoother_turn_point_smoothness_discount_;

      smoother_min_curvature_to_penalize_ = other.smoother_min_curvature_to_penalize_;

      smoother_equal_spacing_weight_ = other.smoother_equal_spacing_weight_;

      smoother_roll_weight_ = other.smoother_roll_weight_;

      smoother_displacement_weight_ = other.smoother_displacement_weight_;

      smoother_traversibility_weight_ = other.smoother_traversibility_weight_;

      smoother_ground_plane_weight_ = other.smoother_ground_plane_weight_;

      smoother_minimum_traversibility_to_search_for_ = other.smoother_minimum_traversibility_to_search_for_;

      smoother_traversibility_threshold_for_no_discount_ = other.smoother_traversibility_threshold_for_no_discount_;

      smoother_hill_climb_gain_ = other.smoother_hill_climb_gain_;

      smoother_gradient_threshold_to_terminate_ = other.smoother_gradient_threshold_to_terminate_;

   }

   /**
            * Unique ID used to identify this message.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Whether or not the planner checks for collisions.
            */
   public void setCheckForCollisions(boolean check_for_collisions)
   {
      check_for_collisions_ = check_for_collisions;
   }
   /**
            * Whether or not the planner checks for collisions.
            */
   public boolean getCheckForCollisions()
   {
      return check_for_collisions_;
   }

   /**
            * Whether the planner computes surface normals. If false, traversibility and roll are also not computed.
            */
   public void setComputeSurfaceNormalCost(boolean compute_surface_normal_cost)
   {
      compute_surface_normal_cost_ = compute_surface_normal_cost;
   }
   /**
            * Whether the planner computes surface normals. If false, traversibility and roll are also not computed.
            */
   public boolean getComputeSurfaceNormalCost()
   {
      return compute_surface_normal_cost_;
   }

   /**
            * Whether the planner computes and checks traversibility.
            */
   public void setComputeTraversibility(boolean compute_traversibility)
   {
      compute_traversibility_ = compute_traversibility;
   }
   /**
            * Whether the planner computes and checks traversibility.
            */
   public boolean getComputeTraversibility()
   {
      return compute_traversibility_;
   }

   /**
            * Whether the body path plan is post-processed with the smoother.
            */
   public void setPerformSmoothing(boolean perform_smoothing)
   {
      perform_smoothing_ = perform_smoothing;
   }
   /**
            * Whether the body path plan is post-processed with the smoother.
            */
   public boolean getPerformSmoothing()
   {
      return perform_smoothing_;
   }

   /**
            * This is the weight assigned to roll in the search space. Increasing this value will decrease the likelihood of the path moving sideways across slopes.
            */
   public void setRollCostWeight(double roll_cost_weight)
   {
      roll_cost_weight_ = roll_cost_weight;
   }
   /**
            * This is the weight assigned to roll in the search space. Increasing this value will decrease the likelihood of the path moving sideways across slopes.
            */
   public double getRollCostWeight()
   {
      return roll_cost_weight_;
   }

   /**
            * The angle in degrees of the deadband applied to the computed roll. Increasing this value decreases the effect small rolls have on the path.
            */
   public void setRollCostDeadband(double roll_cost_deadband)
   {
      roll_cost_deadband_ = roll_cost_deadband;
   }
   /**
            * The angle in degrees of the deadband applied to the computed roll. Increasing this value decreases the effect small rolls have on the path.
            */
   public double getRollCostDeadband()
   {
      return roll_cost_deadband_;
   }

   /**
            * When the roll is below this angle, the cost is linearly discounted to zero. Increasing this value decreases the effect roll has on the path.
            */
   public void setMaxPenalizedRollAngle(double max_penalized_roll_angle)
   {
      max_penalized_roll_angle_ = max_penalized_roll_angle;
   }
   /**
            * When the roll is below this angle, the cost is linearly discounted to zero. Increasing this value decreases the effect roll has on the path.
            */
   public double getMaxPenalizedRollAngle()
   {
      return max_penalized_roll_angle_;
   }

   /**
            * The node height of a vertex is determined as the average height of all cells within this radius.
            */
   public void setSnapRadius(double snap_radius)
   {
      snap_radius_ = snap_radius;
   }
   /**
            * The node height of a vertex is determined as the average height of all cells within this radius.
            */
   public double getSnapRadius()
   {
      return snap_radius_;
   }

   /**
            * When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
            */
   public void setMinSnapHeightThreshold(double min_snap_height_threshold)
   {
      min_snap_height_threshold_ = min_snap_height_threshold;
   }
   /**
            * When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
            */
   public double getMinSnapHeightThreshold()
   {
      return min_snap_height_threshold_;
   }

   /**
            * Weight assigned to minimizing the incline the path takes.
            */
   public void setInclineCostWeight(double incline_cost_weight)
   {
      incline_cost_weight_ = incline_cost_weight;
   }
   /**
            * Weight assigned to minimizing the incline the path takes.
            */
   public double getInclineCostWeight()
   {
      return incline_cost_weight_;
   }

   /**
            * Deadband applied to the incline in the search.
            */
   public void setInclineCostDeadband(double incline_cost_deadband)
   {
      incline_cost_deadband_ = incline_cost_deadband;
   }
   /**
            * Deadband applied to the incline in the search.
            */
   public double getInclineCostDeadband()
   {
      return incline_cost_deadband_;
   }

   /**
            * The maximum incline, in degrees, that is allowed for the body path planner to traverse.
            */
   public void setMaxIncline(double max_incline)
   {
      max_incline_ = max_incline;
   }
   /**
            * The maximum incline, in degrees, that is allowed for the body path planner to traverse.
            */
   public double getMaxIncline()
   {
      return max_incline_;
   }

   /**
            * Width of the collision box used for checking collisions with the environment.
            */
   public void setCollisionBoxSizeY(double collision_box_size_y)
   {
      collision_box_size_y_ = collision_box_size_y;
   }
   /**
            * Width of the collision box used for checking collisions with the environment.
            */
   public double getCollisionBoxSizeY()
   {
      return collision_box_size_y_;
   }

   /**
            * Depth of the collision box used for checking collisions with the environment.
            */
   public void setCollisionBoxSizeX(double collision_box_size_x)
   {
      collision_box_size_x_ = collision_box_size_x;
   }
   /**
            * Depth of the collision box used for checking collisions with the environment.
            */
   public double getCollisionBoxSizeX()
   {
      return collision_box_size_x_;
   }

   /**
            * Intersection height below which collisions are ignored.
            */
   public void setCollisionBoxGroundClearance(double collision_box_ground_clearance)
   {
      collision_box_ground_clearance_ = collision_box_ground_clearance;
   }
   /**
            * Intersection height below which collisions are ignored.
            */
   public double getCollisionBoxGroundClearance()
   {
      return collision_box_ground_clearance_;
   }

   /**
            * Weight placed on maximizing traversibility in the plan.
            */
   public void setTraversibilityWeight(double traversibility_weight)
   {
      traversibility_weight_ = traversibility_weight;
   }
   /**
            * Weight placed on maximizing traversibility in the plan.
            */
   public double getTraversibilityWeight()
   {
      return traversibility_weight_;
   }

   /**
            * Weight placed on the traversibility at the start node when computing the overall traversibility score.
            */
   public void setTraversibilityStanceWeight(double traversibility_stance_weight)
   {
      traversibility_stance_weight_ = traversibility_stance_weight;
   }
   /**
            * Weight placed on the traversibility at the start node when computing the overall traversibility score.
            */
   public double getTraversibilityStanceWeight()
   {
      return traversibility_stance_weight_;
   }

   /**
            * Weight placed on the traversibility at the end node when computing the overall traversibility score.
            */
   public void setTraversibilityStepWeight(double traversibility_step_weight)
   {
      traversibility_step_weight_ = traversibility_step_weight;
   }
   /**
            * Weight placed on the traversibility at the end node when computing the overall traversibility score.
            */
   public double getTraversibilityStepWeight()
   {
      return traversibility_step_weight_;
   }

   /**
            * Minimum score on the start node traversibility to say an edge is traversible.
            */
   public void setMinTraversibilityScore(double min_traversibility_score)
   {
      min_traversibility_score_ = min_traversibility_score;
   }
   /**
            * Minimum score on the start node traversibility to say an edge is traversible.
            */
   public double getMinTraversibilityScore()
   {
      return min_traversibility_score_;
   }

   /**
            * Minimum angle in degrees to penalize in the traversibility score.
            */
   public void setMinNormalAngleToPenalizeForTraversibility(double min_normal_angle_to_penalize_for_traversibility)
   {
      min_normal_angle_to_penalize_for_traversibility_ = min_normal_angle_to_penalize_for_traversibility;
   }
   /**
            * Minimum angle in degrees to penalize in the traversibility score.
            */
   public double getMinNormalAngleToPenalizeForTraversibility()
   {
      return min_normal_angle_to_penalize_for_traversibility_;
   }

   /**
            * Maximum angle in degrees to penalize in the traversibility score.
            */
   public void setMaxNormalAngleToPenalizeForTraversibility(double max_normal_angle_to_penalize_for_traversibility)
   {
      max_normal_angle_to_penalize_for_traversibility_ = max_normal_angle_to_penalize_for_traversibility;
   }
   /**
            * Maximum angle in degrees to penalize in the traversibility score.
            */
   public double getMaxNormalAngleToPenalizeForTraversibility()
   {
      return max_normal_angle_to_penalize_for_traversibility_;
   }

   /**
            * Weight to place on the surface normals when computing the traversibility score.
            */
   public void setTraversibilityInclineWeight(double traversibility_incline_weight)
   {
      traversibility_incline_weight_ = traversibility_incline_weight;
   }
   /**
            * Weight to place on the surface normals when computing the traversibility score.
            */
   public double getTraversibilityInclineWeight()
   {
      return traversibility_incline_weight_;
   }

   /**
            * Box width of cells to include when performing the traversibility calculation.
            */
   public void setTraversibilitySearchWidth(double traversibility_search_width)
   {
      traversibility_search_width_ = traversibility_search_width;
   }
   /**
            * Box width of cells to include when performing the traversibility calculation.
            */
   public double getTraversibilitySearchWidth()
   {
      return traversibility_search_width_;
   }

   /**
            * Minimum number of occupied neighbor cells to say a vertex is traversible.
            */
   public void setMinOccupiedNeighborsForTraversibility(int min_occupied_neighbors_for_traversibility)
   {
      min_occupied_neighbors_for_traversibility_ = min_occupied_neighbors_for_traversibility;
   }
   /**
            * Minimum number of occupied neighbor cells to say a vertex is traversible.
            */
   public int getMinOccupiedNeighborsForTraversibility()
   {
      return min_occupied_neighbors_for_traversibility_;
   }

   /**
            * Half the typical stance width of the robot, used to compute the traversibility.
            */
   public void setHalfStanceWidth(double half_stance_width)
   {
      half_stance_width_ = half_stance_width;
   }
   /**
            * Half the typical stance width of the robot, used to compute the traversibility.
            */
   public double getHalfStanceWidth()
   {
      return half_stance_width_;
   }

   /**
            * Width of the height window of cells to include in the traversibility calculation.
            */
   public void setTraversibilityHeightWindowWidth(double traversibility_height_window_width)
   {
      traversibility_height_window_width_ = traversibility_height_window_width;
   }
   /**
            * Width of the height window of cells to include in the traversibility calculation.
            */
   public double getTraversibilityHeightWindowWidth()
   {
      return traversibility_height_window_width_;
   }

   /**
            * Deadband applied to the height distance of cells in the traversibility calculation.
            */
   public void setTraversibilityHeightWindowDeadband(double traversibility_height_window_deadband)
   {
      traversibility_height_window_deadband_ = traversibility_height_window_deadband;
   }
   /**
            * Deadband applied to the height distance of cells in the traversibility calculation.
            */
   public double getTraversibilityHeightWindowDeadband()
   {
      return traversibility_height_window_deadband_;
   }

   /**
            * Distance to the ground plane estimate that is needed to say the two nodes are in the ground plane.
            */
   public void setHeightProximityForSayingWalkingOnGround(double height_proximity_for_saying_walking_on_ground)
   {
      height_proximity_for_saying_walking_on_ground_ = height_proximity_for_saying_walking_on_ground;
   }
   /**
            * Distance to the ground plane estimate that is needed to say the two nodes are in the ground plane.
            */
   public double getHeightProximityForSayingWalkingOnGround()
   {
      return height_proximity_for_saying_walking_on_ground_;
   }

   /**
            * Minimum discount applied to non-ground cells when computing the traversibility on the ground plane.
            */
   public void setTraversibilityNonGroundDiscountWhenWalkingOnGround(double traversibility_non_ground_discount_when_walking_on_ground)
   {
      traversibility_non_ground_discount_when_walking_on_ground_ = traversibility_non_ground_discount_when_walking_on_ground;
   }
   /**
            * Minimum discount applied to non-ground cells when computing the traversibility on the ground plane.
            */
   public double getTraversibilityNonGroundDiscountWhenWalkingOnGround()
   {
      return traversibility_non_ground_discount_when_walking_on_ground_;
   }

   /**
            * Weight placed on the gradient for avoiding collisions in the smoother.
            */
   public void setSmootherCollisionWeight(double smoother_collision_weight)
   {
      smoother_collision_weight_ = smoother_collision_weight;
   }
   /**
            * Weight placed on the gradient for avoiding collisions in the smoother.
            */
   public double getSmootherCollisionWeight()
   {
      return smoother_collision_weight_;
   }

   /**
            * Weight placed on the gradient for minimizing the angle between successive segments of the body path.
            */
   public void setSmootherSmoothnessWeight(double smoother_smoothness_weight)
   {
      smoother_smoothness_weight_ = smoother_smoothness_weight;
   }
   /**
            * Weight placed on the gradient for minimizing the angle between successive segments of the body path.
            */
   public double getSmootherSmoothnessWeight()
   {
      return smoother_smoothness_weight_;
   }

   /**
            * Discount applied to the smoothness gradients of turn points.
            */
   public void setSmootherTurnPointSmoothnessDiscount(double smoother_turn_point_smoothness_discount)
   {
      smoother_turn_point_smoothness_discount_ = smoother_turn_point_smoothness_discount;
   }
   /**
            * Discount applied to the smoothness gradients of turn points.
            */
   public double getSmootherTurnPointSmoothnessDiscount()
   {
      return smoother_turn_point_smoothness_discount_;
   }

   /**
            * Minimum curvature in degrees to penalize with a gradient.
            */
   public void setSmootherMinCurvatureToPenalize(double smoother_min_curvature_to_penalize)
   {
      smoother_min_curvature_to_penalize_ = smoother_min_curvature_to_penalize;
   }
   /**
            * Minimum curvature in degrees to penalize with a gradient.
            */
   public double getSmootherMinCurvatureToPenalize()
   {
      return smoother_min_curvature_to_penalize_;
   }

   /**
            * Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
            */
   public void setSmootherEqualSpacingWeight(double smoother_equal_spacing_weight)
   {
      smoother_equal_spacing_weight_ = smoother_equal_spacing_weight;
   }
   /**
            * Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
            */
   public double getSmootherEqualSpacingWeight()
   {
      return smoother_equal_spacing_weight_;
   }

   /**
            * Weight placed on the gradient for minimizing the roll cost of the body path plan.
            */
   public void setSmootherRollWeight(double smoother_roll_weight)
   {
      smoother_roll_weight_ = smoother_roll_weight;
   }
   /**
            * Weight placed on the gradient for minimizing the roll cost of the body path plan.
            */
   public double getSmootherRollWeight()
   {
      return smoother_roll_weight_;
   }

   /**
            * Weight placed on a gradient that drives the waypoint towards the initial value.
            */
   public void setSmootherDisplacementWeight(double smoother_displacement_weight)
   {
      smoother_displacement_weight_ = smoother_displacement_weight;
   }
   /**
            * Weight placed on a gradient that drives the waypoint towards the initial value.
            */
   public double getSmootherDisplacementWeight()
   {
      return smoother_displacement_weight_;
   }

   /**
            * Weight placed on the gradient for maximizing the traversibility of the body path plan.
            */
   public void setSmootherTraversibilityWeight(double smoother_traversibility_weight)
   {
      smoother_traversibility_weight_ = smoother_traversibility_weight;
   }
   /**
            * Weight placed on the gradient for maximizing the traversibility of the body path plan.
            */
   public double getSmootherTraversibilityWeight()
   {
      return smoother_traversibility_weight_;
   }

   /**
            * Weight placed on the gradient pushing the body path plan towards cells in the ground plane.
            */
   public void setSmootherGroundPlaneWeight(double smoother_ground_plane_weight)
   {
      smoother_ground_plane_weight_ = smoother_ground_plane_weight;
   }
   /**
            * Weight placed on the gradient pushing the body path plan towards cells in the ground plane.
            */
   public double getSmootherGroundPlaneWeight()
   {
      return smoother_ground_plane_weight_;
   }

   /**
            * Traversibility threshold resulting in a zero gradient for traversibility.
            */
   public void setSmootherMinimumTraversibilityToSearchFor(double smoother_minimum_traversibility_to_search_for)
   {
      smoother_minimum_traversibility_to_search_for_ = smoother_minimum_traversibility_to_search_for;
   }
   /**
            * Traversibility threshold resulting in a zero gradient for traversibility.
            */
   public double getSmootherMinimumTraversibilityToSearchFor()
   {
      return smoother_minimum_traversibility_to_search_for_;
   }

   /**
            * Traversibility threshold above which the traversibility gradient begins to carry less weight.
            */
   public void setSmootherTraversibilityThresholdForNoDiscount(double smoother_traversibility_threshold_for_no_discount)
   {
      smoother_traversibility_threshold_for_no_discount_ = smoother_traversibility_threshold_for_no_discount;
   }
   /**
            * Traversibility threshold above which the traversibility gradient begins to carry less weight.
            */
   public double getSmootherTraversibilityThresholdForNoDiscount()
   {
      return smoother_traversibility_threshold_for_no_discount_;
   }

   /**
            * Gain applied to the smoother gradient for iterative modifications.
            */
   public void setSmootherHillClimbGain(double smoother_hill_climb_gain)
   {
      smoother_hill_climb_gain_ = smoother_hill_climb_gain;
   }
   /**
            * Gain applied to the smoother gradient for iterative modifications.
            */
   public double getSmootherHillClimbGain()
   {
      return smoother_hill_climb_gain_;
   }

   /**
            * Minimum gradient vector magnitude to terminate the smoother iterations.
            */
   public void setSmootherGradientThresholdToTerminate(double smoother_gradient_threshold_to_terminate)
   {
      smoother_gradient_threshold_to_terminate_ = smoother_gradient_threshold_to_terminate;
   }
   /**
            * Minimum gradient vector magnitude to terminate the smoother iterations.
            */
   public double getSmootherGradientThresholdToTerminate()
   {
      return smoother_gradient_threshold_to_terminate_;
   }


   public static Supplier<AStarBodyPathPlannerParametersPacketPubSubType> getPubSubType()
   {
      return AStarBodyPathPlannerParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AStarBodyPathPlannerParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AStarBodyPathPlannerParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_for_collisions_, other.check_for_collisions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.compute_surface_normal_cost_, other.compute_surface_normal_cost_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.compute_traversibility_, other.compute_traversibility_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.perform_smoothing_, other.perform_smoothing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.roll_cost_weight_, other.roll_cost_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.roll_cost_deadband_, other.roll_cost_deadband_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_penalized_roll_angle_, other.max_penalized_roll_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.snap_radius_, other.snap_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_snap_height_threshold_, other.min_snap_height_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incline_cost_weight_, other.incline_cost_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incline_cost_deadband_, other.incline_cost_deadband_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_incline_, other.max_incline_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_box_size_y_, other.collision_box_size_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_box_size_x_, other.collision_box_size_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_box_ground_clearance_, other.collision_box_ground_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_weight_, other.traversibility_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_stance_weight_, other.traversibility_stance_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_step_weight_, other.traversibility_step_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_traversibility_score_, other.min_traversibility_score_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_normal_angle_to_penalize_for_traversibility_, other.min_normal_angle_to_penalize_for_traversibility_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_normal_angle_to_penalize_for_traversibility_, other.max_normal_angle_to_penalize_for_traversibility_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_incline_weight_, other.traversibility_incline_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_search_width_, other.traversibility_search_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_occupied_neighbors_for_traversibility_, other.min_occupied_neighbors_for_traversibility_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.half_stance_width_, other.half_stance_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_height_window_width_, other.traversibility_height_window_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_height_window_deadband_, other.traversibility_height_window_deadband_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_proximity_for_saying_walking_on_ground_, other.height_proximity_for_saying_walking_on_ground_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.traversibility_non_ground_discount_when_walking_on_ground_, other.traversibility_non_ground_discount_when_walking_on_ground_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_collision_weight_, other.smoother_collision_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_smoothness_weight_, other.smoother_smoothness_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_turn_point_smoothness_discount_, other.smoother_turn_point_smoothness_discount_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_min_curvature_to_penalize_, other.smoother_min_curvature_to_penalize_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_equal_spacing_weight_, other.smoother_equal_spacing_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_roll_weight_, other.smoother_roll_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_displacement_weight_, other.smoother_displacement_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_traversibility_weight_, other.smoother_traversibility_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_ground_plane_weight_, other.smoother_ground_plane_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_minimum_traversibility_to_search_for_, other.smoother_minimum_traversibility_to_search_for_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_traversibility_threshold_for_no_discount_, other.smoother_traversibility_threshold_for_no_discount_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_hill_climb_gain_, other.smoother_hill_climb_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_gradient_threshold_to_terminate_, other.smoother_gradient_threshold_to_terminate_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AStarBodyPathPlannerParametersPacket)) return false;

      AStarBodyPathPlannerParametersPacket otherMyClass = (AStarBodyPathPlannerParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.check_for_collisions_ != otherMyClass.check_for_collisions_) return false;

      if(this.compute_surface_normal_cost_ != otherMyClass.compute_surface_normal_cost_) return false;

      if(this.compute_traversibility_ != otherMyClass.compute_traversibility_) return false;

      if(this.perform_smoothing_ != otherMyClass.perform_smoothing_) return false;

      if(this.roll_cost_weight_ != otherMyClass.roll_cost_weight_) return false;

      if(this.roll_cost_deadband_ != otherMyClass.roll_cost_deadband_) return false;

      if(this.max_penalized_roll_angle_ != otherMyClass.max_penalized_roll_angle_) return false;

      if(this.snap_radius_ != otherMyClass.snap_radius_) return false;

      if(this.min_snap_height_threshold_ != otherMyClass.min_snap_height_threshold_) return false;

      if(this.incline_cost_weight_ != otherMyClass.incline_cost_weight_) return false;

      if(this.incline_cost_deadband_ != otherMyClass.incline_cost_deadband_) return false;

      if(this.max_incline_ != otherMyClass.max_incline_) return false;

      if(this.collision_box_size_y_ != otherMyClass.collision_box_size_y_) return false;

      if(this.collision_box_size_x_ != otherMyClass.collision_box_size_x_) return false;

      if(this.collision_box_ground_clearance_ != otherMyClass.collision_box_ground_clearance_) return false;

      if(this.traversibility_weight_ != otherMyClass.traversibility_weight_) return false;

      if(this.traversibility_stance_weight_ != otherMyClass.traversibility_stance_weight_) return false;

      if(this.traversibility_step_weight_ != otherMyClass.traversibility_step_weight_) return false;

      if(this.min_traversibility_score_ != otherMyClass.min_traversibility_score_) return false;

      if(this.min_normal_angle_to_penalize_for_traversibility_ != otherMyClass.min_normal_angle_to_penalize_for_traversibility_) return false;

      if(this.max_normal_angle_to_penalize_for_traversibility_ != otherMyClass.max_normal_angle_to_penalize_for_traversibility_) return false;

      if(this.traversibility_incline_weight_ != otherMyClass.traversibility_incline_weight_) return false;

      if(this.traversibility_search_width_ != otherMyClass.traversibility_search_width_) return false;

      if(this.min_occupied_neighbors_for_traversibility_ != otherMyClass.min_occupied_neighbors_for_traversibility_) return false;

      if(this.half_stance_width_ != otherMyClass.half_stance_width_) return false;

      if(this.traversibility_height_window_width_ != otherMyClass.traversibility_height_window_width_) return false;

      if(this.traversibility_height_window_deadband_ != otherMyClass.traversibility_height_window_deadband_) return false;

      if(this.height_proximity_for_saying_walking_on_ground_ != otherMyClass.height_proximity_for_saying_walking_on_ground_) return false;

      if(this.traversibility_non_ground_discount_when_walking_on_ground_ != otherMyClass.traversibility_non_ground_discount_when_walking_on_ground_) return false;

      if(this.smoother_collision_weight_ != otherMyClass.smoother_collision_weight_) return false;

      if(this.smoother_smoothness_weight_ != otherMyClass.smoother_smoothness_weight_) return false;

      if(this.smoother_turn_point_smoothness_discount_ != otherMyClass.smoother_turn_point_smoothness_discount_) return false;

      if(this.smoother_min_curvature_to_penalize_ != otherMyClass.smoother_min_curvature_to_penalize_) return false;

      if(this.smoother_equal_spacing_weight_ != otherMyClass.smoother_equal_spacing_weight_) return false;

      if(this.smoother_roll_weight_ != otherMyClass.smoother_roll_weight_) return false;

      if(this.smoother_displacement_weight_ != otherMyClass.smoother_displacement_weight_) return false;

      if(this.smoother_traversibility_weight_ != otherMyClass.smoother_traversibility_weight_) return false;

      if(this.smoother_ground_plane_weight_ != otherMyClass.smoother_ground_plane_weight_) return false;

      if(this.smoother_minimum_traversibility_to_search_for_ != otherMyClass.smoother_minimum_traversibility_to_search_for_) return false;

      if(this.smoother_traversibility_threshold_for_no_discount_ != otherMyClass.smoother_traversibility_threshold_for_no_discount_) return false;

      if(this.smoother_hill_climb_gain_ != otherMyClass.smoother_hill_climb_gain_) return false;

      if(this.smoother_gradient_threshold_to_terminate_ != otherMyClass.smoother_gradient_threshold_to_terminate_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AStarBodyPathPlannerParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("check_for_collisions=");
      builder.append(this.check_for_collisions_);      builder.append(", ");
      builder.append("compute_surface_normal_cost=");
      builder.append(this.compute_surface_normal_cost_);      builder.append(", ");
      builder.append("compute_traversibility=");
      builder.append(this.compute_traversibility_);      builder.append(", ");
      builder.append("perform_smoothing=");
      builder.append(this.perform_smoothing_);      builder.append(", ");
      builder.append("roll_cost_weight=");
      builder.append(this.roll_cost_weight_);      builder.append(", ");
      builder.append("roll_cost_deadband=");
      builder.append(this.roll_cost_deadband_);      builder.append(", ");
      builder.append("max_penalized_roll_angle=");
      builder.append(this.max_penalized_roll_angle_);      builder.append(", ");
      builder.append("snap_radius=");
      builder.append(this.snap_radius_);      builder.append(", ");
      builder.append("min_snap_height_threshold=");
      builder.append(this.min_snap_height_threshold_);      builder.append(", ");
      builder.append("incline_cost_weight=");
      builder.append(this.incline_cost_weight_);      builder.append(", ");
      builder.append("incline_cost_deadband=");
      builder.append(this.incline_cost_deadband_);      builder.append(", ");
      builder.append("max_incline=");
      builder.append(this.max_incline_);      builder.append(", ");
      builder.append("collision_box_size_y=");
      builder.append(this.collision_box_size_y_);      builder.append(", ");
      builder.append("collision_box_size_x=");
      builder.append(this.collision_box_size_x_);      builder.append(", ");
      builder.append("collision_box_ground_clearance=");
      builder.append(this.collision_box_ground_clearance_);      builder.append(", ");
      builder.append("traversibility_weight=");
      builder.append(this.traversibility_weight_);      builder.append(", ");
      builder.append("traversibility_stance_weight=");
      builder.append(this.traversibility_stance_weight_);      builder.append(", ");
      builder.append("traversibility_step_weight=");
      builder.append(this.traversibility_step_weight_);      builder.append(", ");
      builder.append("min_traversibility_score=");
      builder.append(this.min_traversibility_score_);      builder.append(", ");
      builder.append("min_normal_angle_to_penalize_for_traversibility=");
      builder.append(this.min_normal_angle_to_penalize_for_traversibility_);      builder.append(", ");
      builder.append("max_normal_angle_to_penalize_for_traversibility=");
      builder.append(this.max_normal_angle_to_penalize_for_traversibility_);      builder.append(", ");
      builder.append("traversibility_incline_weight=");
      builder.append(this.traversibility_incline_weight_);      builder.append(", ");
      builder.append("traversibility_search_width=");
      builder.append(this.traversibility_search_width_);      builder.append(", ");
      builder.append("min_occupied_neighbors_for_traversibility=");
      builder.append(this.min_occupied_neighbors_for_traversibility_);      builder.append(", ");
      builder.append("half_stance_width=");
      builder.append(this.half_stance_width_);      builder.append(", ");
      builder.append("traversibility_height_window_width=");
      builder.append(this.traversibility_height_window_width_);      builder.append(", ");
      builder.append("traversibility_height_window_deadband=");
      builder.append(this.traversibility_height_window_deadband_);      builder.append(", ");
      builder.append("height_proximity_for_saying_walking_on_ground=");
      builder.append(this.height_proximity_for_saying_walking_on_ground_);      builder.append(", ");
      builder.append("traversibility_non_ground_discount_when_walking_on_ground=");
      builder.append(this.traversibility_non_ground_discount_when_walking_on_ground_);      builder.append(", ");
      builder.append("smoother_collision_weight=");
      builder.append(this.smoother_collision_weight_);      builder.append(", ");
      builder.append("smoother_smoothness_weight=");
      builder.append(this.smoother_smoothness_weight_);      builder.append(", ");
      builder.append("smoother_turn_point_smoothness_discount=");
      builder.append(this.smoother_turn_point_smoothness_discount_);      builder.append(", ");
      builder.append("smoother_min_curvature_to_penalize=");
      builder.append(this.smoother_min_curvature_to_penalize_);      builder.append(", ");
      builder.append("smoother_equal_spacing_weight=");
      builder.append(this.smoother_equal_spacing_weight_);      builder.append(", ");
      builder.append("smoother_roll_weight=");
      builder.append(this.smoother_roll_weight_);      builder.append(", ");
      builder.append("smoother_displacement_weight=");
      builder.append(this.smoother_displacement_weight_);      builder.append(", ");
      builder.append("smoother_traversibility_weight=");
      builder.append(this.smoother_traversibility_weight_);      builder.append(", ");
      builder.append("smoother_ground_plane_weight=");
      builder.append(this.smoother_ground_plane_weight_);      builder.append(", ");
      builder.append("smoother_minimum_traversibility_to_search_for=");
      builder.append(this.smoother_minimum_traversibility_to_search_for_);      builder.append(", ");
      builder.append("smoother_traversibility_threshold_for_no_discount=");
      builder.append(this.smoother_traversibility_threshold_for_no_discount_);      builder.append(", ");
      builder.append("smoother_hill_climb_gain=");
      builder.append(this.smoother_hill_climb_gain_);      builder.append(", ");
      builder.append("smoother_gradient_threshold_to_terminate=");
      builder.append(this.smoother_gradient_threshold_to_terminate_);
      builder.append("}");
      return builder.toString();
   }
}
