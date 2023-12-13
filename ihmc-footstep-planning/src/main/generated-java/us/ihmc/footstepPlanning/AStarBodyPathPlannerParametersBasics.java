package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface AStarBodyPathPlannerParametersBasics extends AStarBodyPathPlannerParametersReadOnly, StoredPropertySetBasics
{
   /**
    * whether or not the planner checks for collisions.
    */
   default void setCheckForCollisions(boolean checkForCollisions)
   {
      set(AStarBodyPathPlannerParameters.checkForCollisions, checkForCollisions);
   }

   /**
    * Whether the planner computes surface normals. If false, traversibility and roll
    * are also not computed
    */
   default void setComputeSurfaceNormalCost(boolean computeSurfaceNormalCost)
   {
      set(AStarBodyPathPlannerParameters.computeSurfaceNormalCost, computeSurfaceNormalCost);
   }

   /**
    * Whether the planner computes and checks traversibility
    */
   default void setComputeTraversibility(boolean computeTraversibility)
   {
      set(AStarBodyPathPlannerParameters.computeTraversibility, computeTraversibility);
   }

   /**
    * Whether the body path plan is post-processed with the smoother.
    */
   default void setPerformSmoothing(boolean performSmoothing)
   {
      set(AStarBodyPathPlannerParameters.performSmoothing, performSmoothing);
   }

   /**
    * This is the weight assigned to roll in the search space. Increasing this value
    * will decrease the likelihood of the path moving sideways across slopes.
    */
   default void setRollCostWeight(double rollCostWeight)
   {
      set(AStarBodyPathPlannerParameters.rollCostWeight, rollCostWeight);
   }

   /**
    * The angle in degrees of the deadband applied to the computed roll. Increasing
    * this value decreases the effect small rolls have on the path.
    */
   default void setRollCostDeadband(double rollCostDeadband)
   {
      set(AStarBodyPathPlannerParameters.rollCostDeadband, rollCostDeadband);
   }

   /**
    * When the roll is below this angle, the cost is linearly discounted to zero.
    * Increasing this value decreases the effect roll has on the path.
    */
   default void setMaxPenalizedRollAngle(double maxPenalizedRollAngle)
   {
      set(AStarBodyPathPlannerParameters.maxPenalizedRollAngle, maxPenalizedRollAngle);
   }

   /**
    * The node height of a vertex is determined as the average height of all cells
    * within this radius.
    */
   default void setSnapRadius(double snapRadius)
   {
      set(AStarBodyPathPlannerParameters.snapRadius, snapRadius);
   }

   /**
    * When computing the vertex height, cells that are this distance below the max
    * height are ignored when taking the average
    */
   default void setMinSnapHeightThreshold(double minSnapHeightThreshold)
   {
      set(AStarBodyPathPlannerParameters.minSnapHeightThreshold, minSnapHeightThreshold);
   }

   /**
    * This is the weight assigned to minimizing the inccline the path takes. The cost
    * is determined by the difference between the edge incline from the nominal
    * incline.
    */
   default void setInclineCostWeight(double inclineCostWeight)
   {
      set(AStarBodyPathPlannerParameters.inclineCostWeight, inclineCostWeight);
   }

   /**
    * This is a deadband applied to the incline in the search.
    */
   default void setInclineCostDeadband(double inclineCostDeadband)
   {
      set(AStarBodyPathPlannerParameters.inclineCostDeadband, inclineCostDeadband);
   }

   /**
    * The max incline, in degrees, that is allowed for the body path planner to
    * traverse.
    */
   default void setMaxIncline(double maxIncline)
   {
      set(AStarBodyPathPlannerParameters.maxIncline, maxIncline);
   }

   /**
    * Width of the collision box used for checking collisions with the environment.
    */
   default void setCollisionBoxSizeY(double collisionBoxSizeY)
   {
      set(AStarBodyPathPlannerParameters.collisionBoxSizeY, collisionBoxSizeY);
   }

   /**
    * Depth of the collision box used for checking collisions with the environment.
    */
   default void setCollisionBoxSizeX(double collisionBoxSizeX)
   {
      set(AStarBodyPathPlannerParameters.collisionBoxSizeX, collisionBoxSizeX);
   }

   /**
    * Intersection height below which collisions are ignored.
    */
   default void setCollisionBoxGroundClearance(double collisionBoxGroundClearance)
   {
      set(AStarBodyPathPlannerParameters.collisionBoxGroundClearance, collisionBoxGroundClearance);
   }

   /**
    * Weight placed on maximizing traversibility in the plan. Increasing this weight
    * will tend the plan towards flat sections.
    */
   default void setTraversibilityWeight(double traversibilityWeight)
   {
      set(AStarBodyPathPlannerParameters.traversibilityWeight, traversibilityWeight);
   }

   /**
    * weight placed on the traversibility at the start node when computing the overall
    * traversibility score.
    */
   default void setTraversibilityStanceWeight(double traversibilityStanceWeight)
   {
      set(AStarBodyPathPlannerParameters.traversibilityStanceWeight, traversibilityStanceWeight);
   }

   /**
    * weight placed on the traversibility at the end node when computing the overall
    * traversibility score.
    */
   default void setTraversibilityStepWeight(double traversibilityStepWeight)
   {
      set(AStarBodyPathPlannerParameters.traversibilityStepWeight, traversibilityStepWeight);
   }

   /**
    * Min score on the start node traversibility to say an edge is traversible.
    */
   default void setMinTraversibilityScore(double minTraversibilityScore)
   {
      set(AStarBodyPathPlannerParameters.minTraversibilityScore, minTraversibilityScore);
   }

   /**
    * Min angle in degrees to penalize in the traversibility score.
    */
   default void setMinNormalAngleToPenalizeForTraversibility(double minNormalAngleToPenalizeForTraversibility)
   {
      set(AStarBodyPathPlannerParameters.minNormalAngleToPenalizeForTraversibility, minNormalAngleToPenalizeForTraversibility);
   }

   /**
    * Max angle in degrees to penalize in the traversibility score.
    */
   default void setMaxNormalAngleToPenalizeForTraversibility(double maxNormalAngleToPenalizeForTraversibility)
   {
      set(AStarBodyPathPlannerParameters.maxNormalAngleToPenalizeForTraversibility, maxNormalAngleToPenalizeForTraversibility);
   }

   /**
    * Weight to place on the surface normals when computing the traversibility score.
    */
   default void setTraversibilityInclineWeight(double traversibilityInclineWeight)
   {
      set(AStarBodyPathPlannerParameters.traversibilityInclineWeight, traversibilityInclineWeight);
   }

   /**
    * Box width of cells to include when performing the traversibility calculation
    */
   default void setTraversibilitySearchWidth(double traversibilitySearchWidth)
   {
      set(AStarBodyPathPlannerParameters.traversibilitySearchWidth, traversibilitySearchWidth);
   }

   /**
    * This is the minimum number of occupied neighbor cells to say a vertex is
    * traversibile.
    */
   default void setMinOccupiedNeighborsForTraversibility(int minOccupiedNeighborsForTraversibility)
   {
      set(AStarBodyPathPlannerParameters.minOccupiedNeighborsForTraversibility, minOccupiedNeighborsForTraversibility);
   }

   /**
    * This is half the typical stance width of the robot, used to compute the
    * traversibility
    */
   default void setHalfStanceWidth(double halfStanceWidth)
   {
      set(AStarBodyPathPlannerParameters.halfStanceWidth, halfStanceWidth);
   }

   /**
    * This the width of the height window of cells to include in the traversibility
    * calculation. The height distance must be within this value of the start and end
    * nodes. Increasing this value will increase the number of cells included in the
    * traversibility calculation
    */
   default void setTraversibilityHeightWindowWidth(double traversibilityHeightWindowWidth)
   {
      set(AStarBodyPathPlannerParameters.traversibilityHeightWindowWidth, traversibilityHeightWindowWidth);
   }

   /**
    * This is the deadband applied to the height distance of cells in the
    * traversibility calculation. Increasing this value will increase the overall
    * traversibility scores.
    */
   default void setTraversibilityHeightWindowDeadband(double traversibilityHeightWindowDeadband)
   {
      set(AStarBodyPathPlannerParameters.traversibilityHeightWindowDeadband, traversibilityHeightWindowDeadband);
   }

   /**
    * This is the distance to the ground plane estimate that is needed for saying the
    * two nodes are in the ground plane.
    */
   default void setHeightProximityForSayingWalkingOnGround(double heightProximityForSayingWalkingOnGround)
   {
      set(AStarBodyPathPlannerParameters.heightProximityForSayingWalkingOnGround, heightProximityForSayingWalkingOnGround);
   }

   /**
    * This is the minimum discount applied to non-ground cells when computing the
    * traversibility when the edge is located in the ground plane. Increasing this
    * value will increase the traversibility score when walking on the ground.
    */
   default void setTraversibilityNonGroundDiscountWhenWalkingOnGround(double traversibilityNonGroundDiscountWhenWalkingOnGround)
   {
      set(AStarBodyPathPlannerParameters.traversibilityNonGroundDiscountWhenWalkingOnGround, traversibilityNonGroundDiscountWhenWalkingOnGround);
   }

   /**
    * Weight placed on the gradient for avoiding collisions
    */
   default void setSmootherCollisionWeight(double smootherCollisionWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherCollisionWeight, smootherCollisionWeight);
   }

   /**
    * Weight placed on the gradient for minimizing the angle between successive
    * segments of the body path
    */
   default void setSmootherSmoothnessWeight(double smootherSmoothnessWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherSmoothnessWeight, smootherSmoothnessWeight);
   }

   /**
    * Discount applied to the smoothness gradients of turn points.
    */
   default void setSmootherTurnPointSmoothnessDiscount(double smootherTurnPointSmoothnessDiscount)
   {
      set(AStarBodyPathPlannerParameters.smootherTurnPointSmoothnessDiscount, smootherTurnPointSmoothnessDiscount);
   }

   /**
    * Min curvature in degrees to penalize with a gradient.
    */
   default void setSmootherMinCurvatureToPenalize(double smootherMinCurvatureToPenalize)
   {
      set(AStarBodyPathPlannerParameters.smootherMinCurvatureToPenalize, smootherMinCurvatureToPenalize);
   }

   /**
    * Weight placed on the gradient for making the vertices of the body path plan an
    * equal distance apart.
    */
   default void setSmootherEqualSpacingWeight(double smootherEqualSpacingWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherEqualSpacingWeight, smootherEqualSpacingWeight);
   }

   /**
    * Weight placed on the gradient for minimizing the roll cost of the body path plan
    */
   default void setSmootherRollWeight(double smootherRollWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherRollWeight, smootherRollWeight);
   }

   /**
    * Weight placed on a gradient that drives the waypoint towards the initial value
    */
   default void setSmootherDisplacementWeight(double smootherDisplacementWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherDisplacementWeight, smootherDisplacementWeight);
   }

   /**
    * Weight placed on the gradient for maximizing the traversibility of the body path
    * plan.
    */
   default void setSmootherTraversibilityWeight(double smootherTraversibilityWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherTraversibilityWeight, smootherTraversibilityWeight);
   }

   /**
    * Weight placed on the gradient pushing the body path plan towards the most cells
    * in the ground plane.
    */
   default void setSmootherGroundPlaneWeight(double smootherGroundPlaneWeight)
   {
      set(AStarBodyPathPlannerParameters.smootherGroundPlaneWeight, smootherGroundPlaneWeight);
   }

   /**
    * Traversibility threshold that results in a zero gradient for traversibility
    */
   default void setSmootherMinimumTraversibilityToSearchFor(double smootherMinimumTraversibilityToSearchFor)
   {
      set(AStarBodyPathPlannerParameters.smootherMinimumTraversibilityToSearchFor, smootherMinimumTraversibilityToSearchFor);
   }

   /**
    * Traversibility threshold above which the traversibility gradient begins to carry
    * less weight.
    */
   default void setSmootherTraversibilityThresholdForNoDiscount(double smootherTraversibilityThresholdForNoDiscount)
   {
      set(AStarBodyPathPlannerParameters.smootherTraversibilityThresholdForNoDiscount, smootherTraversibilityThresholdForNoDiscount);
   }

   /**
    * Gain applied to the smoother gradient for iterative modifications
    */
   default void setSmootherHillClimbGain(double smootherHillClimbGain)
   {
      set(AStarBodyPathPlannerParameters.smootherHillClimbGain, smootherHillClimbGain);
   }

   /**
    * Minimum gradient vector magnitude to terminate the smoother iterations
    */
   default void setSmootherGradientThresholdToTerminate(double smootherGradientThresholdToTerminate)
   {
      set(AStarBodyPathPlannerParameters.smootherGradientThresholdToTerminate, smootherGradientThresholdToTerminate);
   }
}
