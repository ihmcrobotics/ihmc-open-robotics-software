package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.graphSearch.stepChecking.ObstacleBetweenStepsChecker;
import us.ihmc.tools.property.StoredPropertySetReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ParameterBasedStepExpansion;

import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.*;

/**
 * Parameters for {@link FootstepPlanningModule}.
 * Parameter sets are saved as .ini files, see atlasFootstepPlannerParameters.ini for an example.
 * For an example of instantiating a java parameters object from an ini file, see AtlasFootstepPlannerParameters(projectName, pathToResources, fileNameSuffix).
 */
public interface FootstepPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   ///////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////         Algorithm parameters       //////////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * Heuristic inflation weight, used to speed up the planner at the expense of optimality
    * Should not be less than 1.0
    */
   default DoubleProvider getAStarHeuristicsWeight()
   {
      return () -> get(aStarHeuristicsWeight);
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
    * If true, enables a mask that reduces the number of calculated steps away from the ideal step. See {@link ParameterBasedStepExpansion} for more information.
    */
   default boolean getEnabledExpansionMask()
   {
      return get(enableExpansionMask);
   }

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////         Ideal footstep parameters       ///////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

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

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////         Footstep restriction parameters       ////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * If true, uses IK-based step feasibility check instead of heuristic one
    */
   default boolean getUseStepReachabilityMap()
   {
      return get(useReachabilityMap);
   }

   /**
    * Solution quality threshold when using IK-based feasibility check, only used when {@link #getUseStepReachabilityMap} is true.
    */
   default double getSolutionQualityThreshold()
   {
      return get(solutionQualityThreshold);
   }

   /**
    * Minimum step width the planner will consider for candidate steps.
    * If this value is too low, for example below the foot's width, the planner could place consecutive footsteps
    * on top of each other. If too high, footsteps might not be kinematically feasible.
    */
   default double getMinimumStepWidth()
   {
      return get(minStepWidth);
   }

   /**
    * Minimum step length the planner will consider for candidate steps.
    * If this value is too low, for example below the foot's length, the planner could place consecutive footsteps
    * on top of each other. If too high, footsteps might not be kinematically feasible.
    */
   default double getMinimumStepLength()
   {
      return get(minStepLength);
   }

   /**
    * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
    * then the value specified here. This value is in radians relative to vertical, so an incline of 0.0 means a flat surface.
    */
   default double getMinimumSurfaceInclineRadians()
   {
      return get(minSurfaceIncline);
   }

   /**
    * Minimum step yaw relative to previous step, where positive yaw is defined as rotating away from the other foot and zero yaw means the feet are parallel.
    */
   default double getMinimumStepYaw()
   {
      return get(minStepYaw);
   }

   /**
    * Maximum vertical distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
    * The maximum depth is determined by linearly interpolating between a step's maximum z value and this value, based on the fraction the foot is pitched by.
    * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
    */
   default double getMinimumStepZWhenFullyPitched()
   {
      return get(minStepZWhenFullyPitched);
   }

   /**
    * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
    * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
    */
   default double getMinimumFootholdPercent()
   {
      return get(minFootholdPercent);
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
    * When using a height map, this returns the maximum accepted RMS error of the best-fit plane.
    */
   default double getRMSErrorThreshold()
   {
      return get(rmsErrorThreshold);
   }

   /**
    * When using a height map, assigns a cost based on the RMS value, which maps (rmsMinErrorToPenalize, rmsErrorThreshold) to (0.0, rmsErrorCost)
    */
   default double getRMSErrorCost()
   {
      return get(rmsErrorCost);
   }

   /**
    * See {@link #getRMSErrorCost()}
    */
   default double getRMSMinErrorToPenalize()
   {
      return get(rmsMinErrorToPenalize);
   }

   /**
    * When using a height map, snapping is done by taking all the points inside the polygon, then removing points below
    * z_max - dz, where z_max is the highest point and dz is this value.
    */
   default double getHeightMapSnapThreshold()
   {
      return get(heightMapSnapThreshold);
   }

   /**
    * Maximum step width the planner will consider for candidate steps.
    * Step width refers to the magnitude of the y-position of a footstep expressed in its parent's sole frame,
    * where the parent is the last footstep taken on the other foot.
    * If this value is too low, the planner will unnecessarily reject footsteps. If too high, footsteps might not be kinematically feasible.
    */
   default double getMaximumStepWidth()
   {
      return get(maxStepWidth);
   }

   /**
    * Maximum xy-distance the planner will consider for candidate steps relative to a squared-up step.
    * A squared-up step is defined by offseting the parent step by the ideal step width.
    */
   default double getMaximumStepReach()
   {
      return get(maxStepReach);
   }

   /**
    * Maximum step yaw relative to previous step, where positive yaw is defined as rotating away from the other foot and zero yaw means the feet are parallel.
    */
   default double getMaximumStepYaw()
   {
      return get(maxStepYaw);
   }

   /**
    * Maximum forward distance between consecutive footsteps when the trailing foot is pitched at {@link #getMinimumSurfaceInclineRadians()} .
    * The maximum distance is determined by linearly interpolating between a step's maximum z value and this value, based on the fraction the foot is pitched by.
    * A candidate footstep will be rejected if its z-value is less than this value, when expressed its parent's z-up sole frame.
    */
   default double getMaximumStepXWhenFullyPitched()
   {
      return get(maxStepXWhenFullyPitched);
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
    * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
    * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
    * it's very close to hitting it's ankle pitch joint limit
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
    * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
    * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
    * it's very close to hitting it's ankle pitch joint limit
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
    * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
    * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
    * it's very close to hitting it's ankle pitch joint limit
    */
   default double getMaximumStepZWhenForwardAndDown()
   {
      return get(maximumStepZWhenForwardAndDown);
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
    * Large steps forward and up can cause the robot to surpass its torque limits.
    * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
    * it's very close to saturating its torque limits.
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
    * Large steps forward and up can cause the robot to surpass its torque limits.
    * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
    * it's very close to saturating its torque limits.
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
    * Large steps forward and up can cause the robot to surpass its torque limits.
    * These parameters should be tuned so that when the robot takes a step of length {@link #getMaximumStepReachWhenSteppingUp()} and {@link #getMaximumStepZWhenSteppingUp()},
    * it's very close to saturating its torque limits.
    */
   default double getMaximumStepZWhenSteppingUp()
   {
      return get(maximumStepZWhenSteppingUp);
   }

   /**
    * Maximum vertical distance between consecutive footsteps, applies for both stepping up and down.
    */
   default double getMaxStepZ()
   {
      return get(maxStepZ);
   }

   /**
    * Maximum vertical distance between start-of-swing and touchdown, applies for both stepping up and down.
    */
   default double getMaxSwingZ()
   {
      return get(maxSwingZ);
   }

   /**
    * Maximum xy distance between start-of-swing and touchdown.
    */
   default double getMaxSwingReach()
   {
      return get(maxSwingReach);
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

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////         Footstep cost parameters       ////////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * Scale factor for cost of deviating from the ideal footstep by rotating.
    * The cost is c*yaw, where c is this weight
    */
   default double getYawWeight()
   {
      return get(yawWeight);
   }

   /**
    * Scale factor for cost of deviating from the ideal footstep by shifting along X, i.e. forwards/backwards
    * The cost is c*stepLength, where c is this weight
    */
   default double getForwardWeight()
   {
      return get(forwardWeight);
   }

   /**
    * Scale factor for cost associated with deviating from the ideal footstep by shifting along Y, i.e. side to side
    * The cost is c*stepWidth, where c is this weight
    */
   default double getLateralWeight()
   {
      return get(lateralWeight);
   }

   /**
    * Constant cost per step to punish to avoid plans with extra steps.
    */
   default double getCostPerStep()
   {
      return get(costPerStep);
   }

   /**
    * Scale factor for cost associated stepping up.
    * The cost is c*stepHeight, where c is this weight
    */
   default double getStepUpWeight()
   {
      return get(stepUpWeight);
   }

   /**
    * Scale factor for cost associated stepping down.
    * The cost is c*abs(stepHeight), where c is this weight
    */
   default double getStepDownWeight()
   {
      return get(stepDownWeight);
   }

   /**
    * Scale factor for cost associated with step rotation.
    * The cost is c*abs(stepRoll), where c is this weight and stepRoll is the step's roll rotation in the parent step's z-up frame
    */
   default double getRollWeight()
   {
      return get(rollWeight);
   }

   /**
    * Scale factor for cost associated with step rotation.
    * The cost is c*abs(stepPitch), where c is this weight and stepPitch is the step's pitch rotation in the parent step's z-up frame
    */
   default double getPitchWeight()
   {
      return get(pitchWeight);
   }

   /**
    * The cost of having a partial foothold.
    * The cost is c * (1 - p)/(1 - p_min), where c is this weight, p is the foothold percentage and p_min is the minimum foothold percentage
    */
   default double getFootholdAreaWeight()
   {
      return get(footholdAreaWeight);
   }

   default double getReferencePlanAlpha()
   {
      return get(referencePlanAlpha);
   }

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////         Footstep snapping and wiggling       /////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

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
    * Maximum height above a stance step that a candidate step is snapped to. Regions above this height are ignored.
    * Intended to avoid ceilings or obstacles that are above the top of the robot
    */
   default double getMaximumSnapHeight()
   {
      return get(maximumSnapHeight);
   }

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////         Body path parameters       //////////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * Radius around the goal inside which the planner should start to turn to match the goal's orientation
    */
   default double getFinalTurnProximity()
   {
      return get(finalTurnProximity);
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

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////         Bounding box collision check       //////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * Sets whether or not the search should check if a bounding box approximation of the robot collides with the world.
    */
   default boolean checkForBodyBoxCollisions()
   {
      return get(checkForBodyBoxCollisions);
   }

   /**
    * Height of body collision box
    */
   default double getBodyBoxHeight()
   {
      return get(bodyBoxHeight);
   }

   /**
    * Depth (along X) of body collision box
    */
   default double getBodyBoxDepth()
   {
      return get(bodyBoxDepth);
   }

   /**
    * Width (along Y) of body collision box
    */
   default double getBodyBoxWidth()
   {
      return get(bodyBoxWidth);
   }

   /**
    * X-offset of collision box relative to a step
    */
   default double getBodyBoxBaseX()
   {
      return get(bodyBoxBaseX);
   }

   /**
    * Y-offset of collision box relative to a step. This centers the collision box and should be roughly half the nominal stance width.
    */
   default double getBodyBoxBaseY()
   {
      return get(bodyBoxBaseY);
   }

   /**
    * Z-offset of collision box relative to a step. If too low the collision box might intersect normal terrain, recommended >50cm
    */
   default double getBodyBoxBaseZ()
   {
      return get(bodyBoxBaseZ);
   }

   /**
    * When {@link #checkForBodyBoxCollisions()} is true, this sets how many bounding box checks to perform.
    * If this value is 1, only the final footstep is checked. Additional checks are done by interpolating
    * between the start and end steps.
    */
   default int getIntermediateBodyBoxChecks()
   {
      return get(intermediateBodyBoxChecks);
   }

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////         Shin collision check       //////////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

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

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////         Other collision-related checks       /////////////////////////////
   ///////////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * Enables a collision check that is lighter-weight than a bounding box. Draws a planar region by vertically extruding the line
    * between consecutive steps and invalidates steps with collisions, see: {@link ObstacleBetweenStepsChecker}
    */
   default boolean checkForPathCollisions()
   {
      return get(checkForPathCollisions);
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

}
