package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.GoodFootstepPositionChecker;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.LinearHeightCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.QuadraticDistanceAndYawCost;
import us.ihmc.tools.property.StoredPropertySetReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.*;

public interface FootstepPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
    * to run slower.
    */
   default boolean checkForBodyBoxCollisions()
   {
      return get(checkForBodyBoxCollisions);
   }

   /**
    * Sets whether or not to perform the defined heuristic search policies.
    */
   default boolean performHeuristicSearchPolicies()
   {
      return get(performHeuristicSearchPolicies);
   }

   /**
    * Returns the ideal step width for walking on flat ground.
    */
   default double getIdealFootstepWidth()
   {
      return get(idealFootstepWidth);
   }

   /**
    * Returns the ideal step length for walking on flat ground.
    */
   default double getIdealFootstepLength()
   {
      return get(idealFootstepLength);
   }

   /**
    * If the planner in use utilized footstep wiggling (see {@link us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler}) to move footholds onto planer
    * regions this parameter will be used. It specifies the minimum distance between the foot polygon and the
    * edge of the planar region polygon that the footstep is moved into. This value can be negative. That corresponds
    * to allowing footsteps that partially intersect planar regions.
    *
    * <p>
    * If this value is too high, the planner will not put footsteps on small planar regions. At zero, the planner might
    * choose a footstep with an edge along a planar region. This value should roughly be set to the sum of two values:
    * <ul>
    *    <li>The smallest acceptable distance to the edge of a cliff</li>
    *    <li>The maximum error between desired and actual foot placement</li>
    * </ul>
    * </p>
    */
   default double getWiggleInsideDelta()
   {
      return get(wiggleInsideDelta);
   }

   /**
    * Maximum xy-distance the planner will consider for candidate steps.
    *
    * <p>
    * Step reach refers to the magnitude of the xy-position of a footstep expressed in its parent's z-up sole frame,
    * where the parent is the last footstep taken on the other foot.
    * </p>
    *
    * <p>
    * This parameter is intended to prevent accepting candidate footsteps that are near both the maximum step length and step width.
    * </p>
    */
   default double getMaximumStepReach()
   {
      return get(maxStepReach);
   }

   /**
    * Maximum yaw between consecutive footsteps
    *
    * <p>
    * A candidate footstep will be rejected if the yaw between it and its parent is greater than this value.
    * </p>
    *
    * <p>
    * This restricts the planner from planning kinematically infeasible footsteps. It is constant through the
    * space of potential steps, so the robot should be able to achieve this yaw, for example, when stepping at
    * its maximum reach.
    * </p>
    */
   default double getMaximumStepYaw()
   {
      return get(maxStepYaw);
   }

   /**
    * Minimum step width the planner will consider for candidate steps.
    *
    * <p>
    * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
    * where the parent is the last footstep taken on the other foot.
    * </p>
    *
    * <p>
    * If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
    * on top of each other. If too high, footsteps might not be kinematically feasible.
    * </p>
    *
    * <p>
    *    The {@link GoodFootstepPositionChecker} will reject a node if it is not wide enough using this parameter.
    * </p>
    */
   default double getMinimumStepWidth()
   {
      return get(minStepWidth);
   }

   /**
    * Minimum step length the planner will consider for candidate steps.
    *
    * <p>
    * Step length refers to the x-position of a footstep expressed in its parent's sole frame,
    * where the parent is the last footstep taken on the other foot.
    * </p>
    *
    * <p>
    * If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
    * on top of each other. If too high, footsteps might not be kinematically feasible.
    * </p>
    */
   default double getMinimumStepLength()
   {
      return get(minStepLength);
   }

   /**
    * Minimum step yaw.
    */
   default double getMinimumStepYaw()
   {
      return get(minStepYaw);
   }

   /**
    * Maximum step reach when stepping up.
    *
    * <p>
    * Long steps forward are rejected by the planner if two criteria are met:
    * <ul>
    *    <li> The x-position of the value of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame. </li>
    * </ul>
    * </p>
    *
    * <p>
    *    Large steps forward and up can cause the robot to surpass its torque limits.
    *    These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
    *    it's very close to saturating its torque limits.
    * </p>
    */
   default double getMaximumStepReachWhenSteppingUp()
   {
      return get(maximumStepReachWhenSteppingUp);
   }

   /**
    * Step height for considering stepping up.
    *
    * <p>
    * Long steps forward are rejected by the planner if two criteria are met:
    * <ul>
    *    <li> The x-position of the value of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> The z-position of the value of the footstep is greater than {@link #getMaximumStepZWhenSteppingUp()}, when expressed in its parent's z-up sole frame. </li>
    * </ul>
    * </p>
    *
    * <p>
    *    Large steps forward and up can cause the robot to surpass its torque limits.
    *    These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
    *    it's very close to saturating its torque limits.
    * </p>
    */
   default double getMaximumStepZWhenSteppingUp()
   {
      return get(maximumStepZWhenSteppingUp);
   }

   /**
    * Maximum step length when stepping forward and down.
    *
    * <p>
    * Large steps forward and down are rejected by the planner if two criteria are met:
    * <ul>
    * <li> The x-position of the value of the footstep exceeds {@link #getMaximumStepXWhenForwardAndDown()}, when expressed in its parent's z-up sole frame </li>
    * <li> The z-position of the value of the footstep is less than -{@link #getMaximumStepZWhenForwardAndDown()}, when expressed in its parent's z-up sole frame </li>
    * </ul>
    * </p>
    *
    * <p>
    * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
    * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
    * it's very close to hitting it's ankle pitch joint limit
    * </p>
    */
   default double getMaximumStepXWhenForwardAndDown()
   {
      return get(maximumStepXWhenForwardAndDown);
   }

   /**
    * Maximum step height when stepping forward and down.
    *
    * <p>
    * Large steps forward and down are rejected by the planner if two criteria are met:
    * <ul>
    * <li> The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame </li>
    * <li> The z-position of the value of the footstep is less than -maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame </li>
    * </ul>
    * </p>
    *
    * <p>
    * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
    * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
    * it's very close to hitting it's ankle pitch joint limit
    * </p>
    */
   default double getMaximumStepZWhenForwardAndDown()
   {
      return get(maximumStepZWhenForwardAndDown);
   }

   /**
    * Maximum vertical distance between consecutive footsteps
    *
    * <p>
    * A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
    * z-up sole frame.
    * </p>
    */
   default double getMaximumStepZ()
   {
      return get(maxStepZ);
   }

   /**
    * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
    * <p>
    * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
    * </p>
    */
   default double getMinimumFootholdPercent()
   {
      return get(minFootholdPercent);
   }

   /**
    * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
    * then the value specified here.
    *
    * <p>
    * More specifically, if a footstep has an associated planar region and that regions surface normal has a
    * z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
    * </p>
    */
   default double getMinimumSurfaceInclineRadians()
   {
      return get(minSurfaceIncline);
   }

   /**
    * There are two methods of wiggling a polygon into a planar region:
    * <ul>
    *    <li>Wiggle the polygon into the planar region itself, which isn't necessarily convex</li>
    *    <li>Wiggle the polygon into the convex hull of the planar region</li>
    * </ul>
    * The first method is not implemented completely. Instead it will wiggle into the sub polygon of the planar region that
    * has the biggest overlap with the foothold.
    *
    * If this parameter is set to true (recommended), the second wiggle method will be used.
    */
   default boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return get(wiggleIntoConvexHullOfPlanarRegions);
   }

   /**
    * If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
    * This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
    * {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
    * candidate footstep will be rejected if this method returns {@code true}.
    */
   default boolean getRejectIfCannotFullyWiggleInside()
   {
      return get(rejectIfCannotFullyWiggleInside);
   }

   /**
    * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
    * distance the planner will use
    */
   default double getMaximumXYWiggleDistance()
   {
      return get(maximumXYWiggleDistance);
   }

   /**
    * When wiggling a candidate footstep into a planar region, this is the maximum yaw
    * distance the planner will use
    */
   default double getMaximumYawWiggle()
   {
      return get(maximumYawWiggle);
   }

   /**
    * When snapping a candidate footstep to a planar region, its possible that another planar region
    * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
    * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
    * otherwise it will.
    */
   default double getMaximumZPenetrationOnValleyRegions()
   {
      return get(maximumZPenetrationOnValleyRegions);
   }

   /**
    * Maximum step width the planner will consider for candidate steps.
    *
    * <p>
    * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
    * where the parent is the last footstep taken on the other foot.
    * </p>
    *
    * <p>
    * If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
    * </p>
    *
    * <p>
    *   The {@link GoodFootstepPositionChecker} will reject a node if it is too wide using this parameter.
    * </p>
    */
   default double getMaximumStepWidth()
   {
      return get(maxStepWidth);
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getCliffHeightToAvoid()
   {
      return get(cliffHeightToAvoid);
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getMinimumDistanceFromCliffBottoms()
   {
      return get(minimumDistanceFromCliffBottoms);
   }

   /**
    * When the planner is done planning and cannot find a path to the goal, this flag indicates whether the
    * planner should return the best plan that it found. If this value is false, the planner will return
    * a {@link FootstepPlan} of type {@link FootstepPlanningResult#NO_PATH_EXISTS}. Otherwise it will return
    * the "best" effort plan, where the plan is at least {@link #getMinimumStepsForBestEffortPlan()} steps long
    * "best" is determined by the planner.
    */
   default boolean getReturnBestEffortPlan()
   {
      return get(returnBestEffortPlan);
   }

   /**
    * When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
    * contains at least this many footsteps.
    */
   default int getMinimumStepsForBestEffortPlan()
   {
      return get(minimumStepsForBestEffortPlan);
   }

   /**
    * Some node checkers will check if the body of the robot will move through a higher planar region
    * (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
    * ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
    * This should be set to be slightly above cinder block height (20.3cm) for Atlas.
    */
   default double getBodyGroundClearance()
   {
      return get(bodyGroundClearance);
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box height. Note that this box will go from {@code getBodyBoxBaseZ}
    * to {@code getBodyBoxBaseHeight + getBodyBoxHeight}
    */
   default double getBodyBoxHeight()
   {
      return get(bodyBoxHeight);
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box depth.
    */
   default double getBodyBoxDepth()
   {
      return get(bodyBoxDepth);
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box width.
    */
   default double getBodyBoxWidth()
   {
      return get(bodyBoxWidth);
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseX()
   {
      return get(bodyBoxBaseX);
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseY()
   {
      return get(bodyBoxBaseY);
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseZ()
   {
      return get(bodyBoxBaseZ);
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   default double getMinXClearanceFromStance()
   {
      return get(minXClearanceFromStance);
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   default double getMinYClearanceFromStance()
   {
      return get(minYClearanceFromStance);
   }

   /**
    * Radius around the goal inside which the planner should start to turn to match the goal's orientation
    */
   default double getFinalTurnProximity()
   {
      return get(finalTurnProximity);
   }

   /**
    * Radius around the goal inside which the body path heuristic planner should start to turn to match the goal's orientation
    */
   default double getFinalTurnBodyPathProximity()
   {
      return get(finalTurnBodyPathProximity);
   }

   /**
    * Defines a percentage of the radius around the final turn proximity in which the blending from the desired heading to the final orientation should occur.
    * That is, at 1 + {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()}, the desired orientation is the desired heading,
    * and at 1 - {@link #getFinalTurnProximityBlendFactor()}} * {@link #getFinalTurnProximity()}, the desired orientation is the final orientation.
    */
   default double getFinalTurnProximityBlendFactor()
   {
      return get(finalTurnProximityBlendFactor);
   }

   /**
    * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
    */
   default boolean useQuadraticDistanceCost()
   {
      return get(useQuadraticDistanceCost);
   }

   /**
    * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
    */
   default boolean useQuadraticHeightCost()
   {
      return get(useQuadraticHeightCost);
   }

   /**
    * Gets the weight for the heuristics in the A Star planner.
    */
   default DoubleProvider getAStarHeuristicsWeight()
   {
      return () -> get(aStarHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Visibility graph with A star planner.
    */
   default DoubleProvider getVisGraphWithAStarHeuristicsWeight()
   {
      return () -> get(visGraphWithAStarHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Depth First planner.
    */
   default DoubleProvider getDepthFirstHeuristicsWeight()
   {
      return () -> get(depthFirstHeuristicsWeight);
   }

   /**
    * Gets the weight for the heuristics in the Body path based planner.
    */
   default DoubleProvider getBodyPathBasedHeuristicsWeight()
   {
      return () -> get(bodyPathBasedHeuristicsWeight);
   }

   /**
    * When using a cost based planning approach this value defined how the yaw of a footstep will be
    * weighted in comparison to its position.
    */
   default double getYawWeight()
   {
      return get(yawWeight);
   }

   /**
    * <p>
    * This value defined how the forward (or backward) displacement of a footstep will be weighted in
    * comparison to its position.
    * </p>
    * <p>
    *    Note that when using a Euclidean distance, this weight is averaged with the value returned by
    *    {@link #getLateralWeight()}
    * </p>
    */
   default double getForwardWeight()
   {
      return get(forwardWeight);
   }

   /**
    * <p>
    * This value defined how the lateral displacement of a footstep will be weighted in comparison to
    * its position.
    * </p>
    * <p>
    *    Note that when using a Euclidean distance, this weight is averaged with the value returned by
    *    {@link #getForwardWeight()}
    * </p>
    */
   default double getLateralWeight()
   {
      return get(lateralWeight);
   }

   /**
    * When using a cost based planning approach this value defines the cost that is added for each step
    * taken. Setting this value to a high number will favor plans with less steps.
    */
   default double getCostPerStep()
   {
      return get(costPerStep);
   }

   /**
    * When using a cost based planning approach this value defines how the height change when stepping
    * up will be weighted.
    */
   default double getStepUpWeight()
   {
      return get(stepUpWeight);
   }

   /**
    * When using a cost based planning approach this value defines how the height change when stepping
    * down will be weighted.
    */
   default double getStepDownWeight()
   {
      return get(stepDownWeight);
   }

   /**
    * When using a cost based planning approach this value defines how the roll will be weighted.
    */
   default double getRollWeight()
   {
      return get(rollWeight);
   }

   /**
    * When using a cost based planning approach this value defines how the pitch will be weighted.
    */
   default double getPitchWeight()
   {
      return get(pitchWeight);
   }

   /**
    * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
    * @see FootstepPlannerCostParameters#getBoundingBoxCost
    */
   default double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return get(maximum2dDistanceFromBoundingBoxToPenalize);

   }

   /**
    * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
    * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
    * {@code c * (1 - d / d_max)}, where d_max is this value.
    */
   default double getBoundingBoxCost()
   {
      return get(boundingBoxCost);
   }

   /**
    * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
    */
   default double getFootholdAreaWeight()
   {
      return get(footholdAreaWeight);
   }

   /**
    * When using a cost based planning approach this value defines the weight of the step distance of a step longer than {@link FootstepPlannerParameters#getIdealFootstepLength()} .
    */
   default double getLongStepWeight()
   {
      return get(longStepWeight);
   }

   default double getBodyPathViolationWeight()
   {
      return get(bodyPathViolationWeight);
   }

   default double getDistanceFromPathTolerance()
   {
      return get(distanceFromPathTolerance);
   }

   default double getDeltaYawFromReferenceTolerance()
   {
      return get(deltaYawFromReferenceTolerance);
   }

   /**
    * Parameters for setting swing trajectories from footstep poses. Will use default values if this returns null
    */
   default AdaptiveSwingParameters getAdaptiveSwingParameters()
   {
      return null;
   }
}