package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface AStarBodyPathPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * whether or not the planner checks for collisions.
    */
   default boolean getCheckForCollisions()
   {
      return get(checkForCollisions);
   }

   /**
    * Whether the planner computes surface normals. If false, traversibility and roll
    * are also not computed
    */
   default boolean getComputeSurfaceNormalCost()
   {
      return get(computeSurfaceNormalCost);
   }

   /**
    * Whether the planner computes and checks traversibility
    */
   default boolean getComputeTraversibility()
   {
      return get(computeTraversibility);
   }

   /**
    * Whether the body path plan is post-processed with the smoother.
    */
   default boolean getPerformSmoothing()
   {
      return get(performSmoothing);
   }

   /**
    * This is the weight assigned to roll in the search space. Increasing this value
    * will decrease the likelihood of the path moving sideways across slopes.
    */
   default double getRollCostWeight()
   {
      return get(rollCostWeight);
   }

   /**
    * The angle in degrees of the deadband applied to the computed roll. Increasing
    * this value decreases the effect small rolls have on the path.
    */
   default double getRollCostDeadband()
   {
      return get(rollCostDeadband);
   }

   /**
    * When the roll is below this angle, the cost is linearly discounted to zero.
    * Increasing this value decreases the effect roll has on the path.
    */
   default double getMaxPenalizedRollAngle()
   {
      return get(maxPenalizedRollAngle);
   }

   /**
    * The node height of a vertex is determined as the average height of all cells
    * within this radius.
    */
   default double getSnapRadius()
   {
      return get(snapRadius);
   }

   /**
    * When computing the vertex height, cells that are this distance below the max
    * height are ignored when taking the average
    */
   default double getMinSnapHeightThreshold()
   {
      return get(minSnapHeightThreshold);
   }

   /**
    * This is the weight assigned to minimizing the inccline the path takes. The cost
    * is determined by the difference between the edge incline from the nominal
    * incline.
    */
   default double getInclineCostWeight()
   {
      return get(inclineCostWeight);
   }

   /**
    * This is a deadband applied to the incline in the search.
    */
   default double getInclineCostDeadband()
   {
      return get(inclineCostDeadband);
   }

   /**
    * The max incline, in degrees, that is allowed for the body path planner to
    * traverse.
    */
   default double getMaxIncline()
   {
      return get(maxIncline);
   }

   /**
    * Width of the collision box used for checking collisions with the environment.
    */
   default double getCollisionBoxSizeY()
   {
      return get(collisionBoxSizeY);
   }

   /**
    * Depth of the collision box used for checking collisions with the environment.
    */
   default double getCollisionBoxSizeX()
   {
      return get(collisionBoxSizeX);
   }

   /**
    * Intersection height below which collisions are ignored.
    */
   default double getCollisionBoxGroundClearance()
   {
      return get(collisionBoxGroundClearance);
   }

   /**
    * Weight placed on maximizing traversibility in the plan. Increasing this weight
    * will tend the plan towards flat sections.
    */
   default double getTraversibilityWeight()
   {
      return get(traversibilityWeight);
   }

   /**
    * weight placed on the traversibility at the start node when computing the overall
    * traversibility score.
    */
   default double getTraversibilityStanceWeight()
   {
      return get(traversibilityStanceWeight);
   }

   /**
    * weight placed on the traversibility at the end node when computing the overall
    * traversibility score.
    */
   default double getTraversibilityStepWeight()
   {
      return get(traversibilityStepWeight);
   }

   /**
    * Min score on the start node traversibility to say an edge is traversible.
    */
   default double getMinTraversibilityScore()
   {
      return get(minTraversibilityScore);
   }

   /**
    * Min angle in degrees to penalize in the traversibility score.
    */
   default double getMinNormalAngleToPenalizeForTraversibility()
   {
      return get(minNormalAngleToPenalizeForTraversibility);
   }

   /**
    * Max angle in degrees to penalize in the traversibility score.
    */
   default double getMaxNormalAngleToPenalizeForTraversibility()
   {
      return get(maxNormalAngleToPenalizeForTraversibility);
   }

   /**
    * Weight to place on the surface normals when computing the traversibility score.
    */
   default double getTraversibilityInclineWeight()
   {
      return get(traversibilityInclineWeight);
   }

   /**
    * Box width of cells to include when performing the traversibility calculation
    */
   default double getTraversibilitySearchWidth()
   {
      return get(traversibilitySearchWidth);
   }

   /**
    * This is the minimum number of occupied neighbor cells to say a vertex is
    * traversibile.
    */
   default int getMinOccupiedNeighborsForTraversibility()
   {
      return get(minOccupiedNeighborsForTraversibility);
   }

   /**
    * This is half the typical stance width of the robot, used to compute the
    * traversibility
    */
   default double getHalfStanceWidth()
   {
      return get(halfStanceWidth);
   }

   /**
    * This the width of the height window of cells to include in the traversibility
    * calculation. The height distance must be within this value of the start and end
    * nodes. Increasing this value will increase the number of cells included in the
    * traversibility calculation
    */
   default double getTraversibilityHeightWindowWidth()
   {
      return get(traversibilityHeightWindowWidth);
   }

   /**
    * This is the deadband applied to the height distance of cells in the
    * traversibility calculation. Increasing this value will increase the overall
    * traversibility scores.
    */
   default double getTraversibilityHeightWindowDeadband()
   {
      return get(traversibilityHeightWindowDeadband);
   }

   /**
    * This is the distance to the ground plane estimate that is needed for saying the
    * two nodes are in the ground plane.
    */
   default double getHeightProximityForSayingWalkingOnGround()
   {
      return get(heightProximityForSayingWalkingOnGround);
   }

   /**
    * This is the minimum discount applied to non-ground cells when computing the
    * traversibility when the edge is located in the ground plane. Increasing this
    * value will increase the traversibility score when walking on the ground.
    */
   default double getTraversibilityNonGroundDiscountWhenWalkingOnGround()
   {
      return get(traversibilityNonGroundDiscountWhenWalkingOnGround);
   }

   /**
    * Weight placed on the gradient for avoiding collisions
    */
   default double getSmootherCollisionWeight()
   {
      return get(smootherCollisionWeight);
   }

   /**
    * Weight placed on the gradient for minimizing the angle between successive
    * segments of the body path
    */
   default double getSmootherSmoothnessWeight()
   {
      return get(smootherSmoothnessWeight);
   }

   /**
    * Discount applied to the smoothness gradients of turn points.
    */
   default double getSmootherTurnPointSmoothnessDiscount()
   {
      return get(smootherTurnPointSmoothnessDiscount);
   }

   /**
    * Min curvature in degrees to penalize with a gradient.
    */
   default double getSmootherMinCurvatureToPenalize()
   {
      return get(smootherMinCurvatureToPenalize);
   }

   /**
    * Weight placed on the gradient for making the vertices of the body path plan an
    * equal distance apart.
    */
   default double getSmootherEqualSpacingWeight()
   {
      return get(smootherEqualSpacingWeight);
   }

   /**
    * Weight placed on the gradient for minimizing the roll cost of the body path plan
    */
   default double getSmootherRollWeight()
   {
      return get(smootherRollWeight);
   }

   /**
    * Weight placed on a gradient that drives the waypoint towards the initial value
    */
   default double getSmootherDisplacementWeight()
   {
      return get(smootherDisplacementWeight);
   }

   /**
    * Weight placed on the gradient for maximizing the traversibility of the body path
    * plan.
    */
   default double getSmootherTraversibilityWeight()
   {
      return get(smootherTraversibilityWeight);
   }

   /**
    * Weight placed on the gradient pushing the body path plan towards the most cells
    * in the ground plane.
    */
   default double getSmootherGroundPlaneWeight()
   {
      return get(smootherGroundPlaneWeight);
   }

   /**
    * Traversibility threshold that results in a zero gradient for traversibility
    */
   default double getSmootherMinimumTraversibilityToSearchFor()
   {
      return get(smootherMinimumTraversibilityToSearchFor);
   }

   /**
    * Traversibility threshold above which the traversibility gradient begins to carry
    * less weight.
    */
   default double getSmootherTraversibilityThresholdForNoDiscount()
   {
      return get(smootherTraversibilityThresholdForNoDiscount);
   }

   /**
    * Gain applied to the smoother gradient for iterative modifications
    */
   default double getSmootherHillClimbGain()
   {
      return get(smootherHillClimbGain);
   }

   /**
    * Minimum gradient vector magnitude to terminate the smoother iterations
    */
   default double getSmootherGradientThresholdToTerminate()
   {
      return get(smootherGradientThresholdToTerminate);
   }
}
