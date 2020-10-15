package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseChecker;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.ObstacleBetweenStepsChecker;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.property.StoredPropertySetReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.*;

public interface FootstepPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Sets whether or not the search should check if a bounding box approximation of the robot collides with the world.
    * This may cause the planner to run slower.
    */
   default boolean checkForBodyBoxCollisions()
   {
      return get(checkForBodyBoxCollisions);
   }

   /**
    * Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
    * between consecutive steps and invalidates steps with collisions, see: {@link ObstacleBetweenStepsChecker}
    */
   default boolean checkForPathCollisions()
   {
      return get(checkForPathCollisions);
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
    * Returns the ideal step width when "shuffling" sideways.
    */
   default double getIdealSideStepWidth()
   {
      return get(idealSideStepWidth);
   }

   /**
    * Returns the ideal length when walking backwards. This value is positive.
    */
   default double getIdealBackStepLength()
   {
      return get(idealBackStepLength);
   }

   /**
    * Returns ideal step length when the vertical height between the start-of-swing and stance feet are at maximum allowed height.
    * Ideal step length is linearly interpolated between {@link #getIdealFootstepLength} on flat ground and this value at max stance height
    */
   default double getIdealStepLengthAtMaxStepZ()
   {
      return get(idealStepLengthAtMaxStepZ);
   }

   /**
    * The planner will try to shift footsteps inside of a region so that this value is the minimum distance from the step
    * to the edge. A negative value means the footstep can overhang a region.
    */
   default double getWiggleInsideDeltaTarget()
   {
      return get(wiggleInsideDeltaTarget);
   }

   /**
    * This parameter only is used if {@link #getWiggleWhilePlanning} is true. If a step cannot be wiggled inside by this amount or more,
    * it will be rejected. Note that if {@link #getWiggleWhilePlanning} if false, it's always best effort on the final plan.
    */
   default double getWiggleInsideDeltaMinimum()
   {
      return get(wiggleInsideDeltaMinimum);
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
    *    The {@link FootstepPoseChecker} will reject a node if it is not wide enough using this parameter.
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
    * Long steps forward are rejected by the planner if one of two criteria are met:
    * <ul>
    *    <li> The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> - OR - </li>
    *    <li> The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()} ()}, when expressed in its parent's z-up sole frame </li>
    *    <li> - AND - </li>
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
    * Maximum step width when stepping up.
    *
    * <p>
    * Long steps forward are rejected by the planner if one of two criteria are met:
    * <ul>
    *    <li> The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> - OR - </li>
    *    <li> The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> - AND - </li>
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
   default double getMaximumStepWidthWhenSteppingUp()
   {
      return get(maximumStepWidthWhenSteppingUp);
   }

   /**
    * Step height for considering stepping up.
    *
    * <p>
    * Long steps forward are rejected by the planner if one of two criteria are met:
    * <ul>
    *    <li> The total length of the footstep exceeds {@link #getMaximumStepReachWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> - OR - </li>
    *    <li> The y-position of the value of the footstep exceeds {@link #getMaximumStepWidthWhenSteppingUp()}, when expressed in its parent's z-up sole frame </li>
    *    <li> - AND - </li>
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
    * Large steps forward and down are rejected by the planner if one of two criteria are met:
    * <ul>
    * <li> The x-position of the value of the footstep exceeds {@link #getMaximumStepXWhenForwardAndDown()}, when expressed in its parent's z-up sole frame </li>
    * <li> - OR - </li>
    * <li> The y-position of the value of the footstep exceeds {@link #getMaximumStepYWhenForwardAndDown()}, when expressed in its parent's z-up sole frame </li>
    * <li> - AND - </li>
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
    * Maximum step width when stepping forward and down.
    *
    * <p>
    * Large steps forward and down are rejected by the planner if one of two criteria are met:
    * <ul>
    * <li> The x-position of the value of the footstep exceeds {@link #getMaximumStepXWhenForwardAndDown()}, when expressed in its parent's z-up sole frame </li>
    * <li> - OR - </li>
    * <li> The y-position of the value of the footstep exceeds {@link #getMaximumStepYWhenForwardAndDown()}, when expressed in its parent's z-up sole frame </li>
    * <li> - AND - </li>
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
   default double getMaximumStepYWhenForwardAndDown()
   {
      return get(maximumStepYWhenForwardAndDown);
   }

   /**
    * Maximum step height when stepping forward and down.
    *
    * <p>
    * Large steps forward and down are rejected by the planner if one of two criteria are met:
    * <ul>
    * <li> The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame </li>
    * <li> - OR - </li>
    * <li> The y-position of the value of the footstep exceeds maximumStepYWhenForwardAndDown, when expressed in its parent's z-up sole frame </li>
    * <li> - AND - </li>
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
   default double getMaxStepZ()
   {
      return get(maxStepZ);
   }

   /**
    * Maximum vertical distance between start-of-swing and touchdown
    */
   default double getMaxSwingZ()
   {
      return get(maxSwingZ);
   }

   /**
    * Maximum xy distance between start-of-swing and touchdown
    */
   default double getMaxSwingReach()
   {
      return get(maxSwingReach);
   }

   /**
    * Maximum vertical distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
    *
    * <p>
    *    The maximum depth is determined by linearly interpolating between a step's maximum z value and this value, based on the fraction the foot is pitched by.
    * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
    * </p>
    */
   default double getMinimumStepZWhenFullyPitched()
   {
      return get(minStepZWhenFullyPitched);
   }

   /**
    * Maximum forward distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
    *
    * <p>
    *    The maximum depdistanceth is determined by linearly interpolating between a step's maximum z value and this value, based on the fraction the foot is pitched by.
    * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
    * </p>
    */
   default double getMaximumStepXWhenFullyPitched()
   {
      return get(maxStepXWhenFullyPitched);
   }

   /**
    * This is the reduction factor for the max yaw when the step is at max reach.
    * This means that, when the footstep is at its maximum distance, this is the fraction reduction of the max yaw.
    * If this returns 0.0, the max yaw is not modified, even at full reach.
    * If this returns 1.0, the max yaw is 0 at full reach.
    *
    * That is,
    * modifiedMaxYaw = (1.0 - reach / maxReach) * maxYaw + reach / maxReach * (1.0 - alpha) * maxYaw
    *
    * @return alpha in the above equation
    */
   default double getStepYawReductionFactorAtMaxReach()
   {
      return get(stepYawReductionFactorAtMaxReach);
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
    * There are two solvers for wiggling the step, one constrains to the region's convex hull and the other to the region's concave hull,
    * this toggles between them.
    */
   default boolean getEnableConcaveHullWiggler()
   {
      return get(enableConcaveHullWiggler);
   }

   /**
    * The wiggler can either run as a post-processor on a resulting plan or on each candidate step while planning.
    */
   default boolean getWiggleWhilePlanning()
   {
      return get(wiggleWhilePlanning);
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
    *   The {@link FootstepPoseChecker} will reject a node if it is too wide using this parameter.
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
   default double getCliffBaseHeightToAvoid()
   {
      return get(cliffBaseHeightToAvoid);
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is {@link #getCliffBaseHeightToAvoid} higher than the candidate footstep, it will move away from it
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
    * The planner can be setup to avoid footsteps near the top of "cliffs". When the footstep has a planar region
    * nearby that is cliffTopHeightToAvoid higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffTops away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getCliffTopHeightToAvoid()
   {
      return get(cliffTopHeightToAvoid);
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is {@link #getCliffTopHeightToAvoid} higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getMinimumDistanceFromCliffTops()
   {
      return get(minimumDistanceFromCliffTops);
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
    * Maximum height above a stance step that a candidate step is snapped to. Regions above this height are ignored.
    * Intended to avoid ceilings or obstacles that are above the top of the robot
    */
   default double getMaximumSnapHeight()
   {
      return get(maximumSnapHeight);
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   default double getMinClearanceFromStance()
   {
      return get(minClearanceFromStance);
   }

   /**
    * Radius around the goal inside which the planner should start to turn to match the goal's orientation
    */
   default double getFinalTurnProximity()
   {
      return get(finalTurnProximity);
   }

   /**
    * Gets the weight for the heuristics in the A Star planner.
    */
   default DoubleProvider getAStarHeuristicsWeight()
   {
      return () -> get(aStarHeuristicsWeight);
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
    */
   default double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return get(maximum2dDistanceFromBoundingBoxToPenalize);

   }

   /**
    * When {@link #checkForBodyBoxCollisions()} is true, this sets how many bounding box checks to perform.
    * If this value is 1, only the final footstep is checked. Additional checks are done by interpolating
    * between the start and end steps.
    */
   default int getNumberOfBoundingBoxChecks()
   {
      return get(numberOfBoundingBoxChecks);
   }

   /**
    * When using a cost based planning approach this value defines how the a missing foothold area will be weighted.
    */
   default double getFootholdAreaWeight()
   {
      return get(footholdAreaWeight);
   }

   /**
    * If the robot's mid-foot pose is within this distance of the body path, it will match the body path heading.
    * Otherwise, it will turn towards the body path
    */
   default double getDistanceFromPathTolerance()
   {
      return get(distanceFromPathTolerance);
   }

   /**
    * If the robot's mid-foot pose oriented within this threshold of the body path's heading, it will match the body path heading.
    * Otherwise, it will turn in plance towards the body path
    */
   default double getDeltaYawFromReferenceTolerance()
   {
      return get(deltaYawFromReferenceTolerance);
   }

   /**
    * Maximum steps considered at each iteration. If more than this number of steps are available, the closest steps to the
    * ideal step are considered and the others are ignored. Set to non-positive number to disable
    */
   default int getMaximumBranchFactor()
   {
      return get(maximumBranchFactor);
   }

   /**
    * If true, enables a mask that reduces the number of calculated steps away from the ideal step
    */
   default boolean getEnabledExpansionMask()
   {
      return get(enableExpansionMask);
   }

   /**
    * If true will try to wiggle steps away from shin collisions. Collisions are checked against all regions.
    * Enable concave hull wiggler must be true in order for the shin collision checker to run.
    */
   default boolean getEnableShinCollisionCheck()
   {
      return get(enableShinCollisionCheck);
   }

   /**
    * How far the shin collision cylinder extends from the toe
    */
   default double getShinToeClearance()
   {
      return get(shinToeClearance);
   }

   /**
    * How far the shin collision cylinder extends from the heel
    */
   default double getShinHeelClearance()
   {
      return get(shinHeelClearance);
   }

   /**
    * Length of the shin collidable cylinder
    */
   default double getShinLength()
   {
      return get(shinLength);
   }

   /**
    * Height offset of shin collidable cylinder
    */
   default double getShinHeightOffset()
   {
      return get(shinHeightOffet);
   }
}
