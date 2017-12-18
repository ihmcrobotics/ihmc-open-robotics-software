package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;

public interface FootstepPlannerParameters
{
   /**
    * Returns the ideal step width for walking on flat ground.
    */
   public abstract double getIdealFootstepWidth();

   /**
    * Returns the ideal step length for walking on flat ground.
    */
   public abstract double getIdealFootstepLength();

   /**
    * If the planner in use utilized footstep wiggling (see {@link PolygonWiggler}) to move footholds onto planer
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
   public default double getWiggleInsideDelta()
   {
      return 0.0;
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
   public abstract double getMaximumStepReach();

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
   public abstract double getMaximumStepYaw();

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
    */
   public abstract double getMinimumStepWidth();

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
   public default double getMinimumStepLength()
   {
      return -getMaximumStepReach();
   }

   /**
    * Minimum step yaw.
    */
   public default double getMinimumStepYaw()
   {
      return 0.0;
   }

   /**
    * Maximum step length when stepping forward and down.
    *
    * <p>
    * Large steps forward and down are rejected by the planner if two criteria are met:
    * <ul>
    * <li> The x-position of the value of the footstep exceeds maximumStepXWhenForwardAndDown, when expressed in its parent's z-up sole frame </li>
    * <li> The z-position of the value of the footstep is less than maximumStepZWhenForwardAndDown, when expressed in its parent's z-up sole frame </li>
    * </ul>
    * </p>
    *
    * <p>
    * Large steps forward and down usually cause the robot to hit it's ankle pitch joint limit.
    * These parameters should be tuned so that when the robot takes a step of length maximumStepXWhenForwardAndDown and height maximumStepZWhenForwardAndDown,
    * it's very close to hitting it's ankle pitch joint limit
    * </p>
    */
   public default double getMaximumStepXWhenForwardAndDown()
   {
      return Double.POSITIVE_INFINITY;
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
   public default double getMaximumStepZWhenForwardAndDown()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * Maximum vertical distance between consecutive footsteps
    *
    * <p>
    * A candidate footstep will be rejected if its z-value is greater than this value, when expressed its parent's
    * z-up sole frame.
    * </p>
    */
   public abstract double getMaximumStepZ();

   /**
    * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
    * <p>
    * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
    * </p>
    */
   public default double getMinimumFootholdPercent()
   {
      return 0.95;
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
   public default double getMinimumSurfaceInclineRadians()
   {
      return Math.toRadians(45.0);
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
   public default boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return true;
   }

   /**
    * If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
    * This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
    * {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
    * candidate footstep will be rejected if this method returns {@code true}.
    */
   public default boolean getRejectIfCannotFullyWiggleInside()
   {
      return false;
   }

   /**
    * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
    * distance the planner will use
    */
   public default double getMaximumXYWiggleDistance()
   {
      return FootstepNode.gridSizeXY / 2.0;
   }

   /**
    * When wiggling a candidate footstep into a planar region, this is the maximum yaw
    * distance the planner will use
    */
   public default double getMaximumYawWiggle()
   {
      return FootstepNode.gridSizeYaw / 2.0;
   }

   /**
    * When snapping a candidate footstep to a planar region, its possible that another planar region
    * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
    * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
    * otherwise it will.
    */
   public default double getMaximumZPenetrationOnValleyRegions()
   {
      return Double.POSITIVE_INFINITY;
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
    */
   public abstract double getMaximumStepWidth();

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
   public default double getCliffHeightToAvoid()
   {
      return Double.MAX_VALUE;
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
   public default double getMinimumDistanceFromCliffBottoms()
   {
      return 0.0;
   }

   /**
    * When the planner is done planning and cannot find a path to the goal, this flag indicates whether the
    * planner should return the best plan that it found. If this value is false, the planner will return
    * a {@link FootstepPlan} of type {@link FootstepPlanningResult#NO_PATH_EXISTS}. Otherwise it will return
    * the "best" effort plan, where the plan is at least {@link #getMinimumStepsForBestEffortPlan()} steps long
    * "best" is determined by the planner.
    */
   public default boolean getReturnBestEffortPlan()
   {
      return false;
   }

   /**
    * When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
    * contains at least this many footsteps.
    */
   public default int getMinimumStepsForBestEffortPlan()
   {
      return 0;
   }

   /**
    * When using a cost based planning approach this value defined how the yaw of a footstep will be
    * weighted in comparison to its position.
    */
   public default double getYawWeight()
   {
      return 0.1;
   }

   /**
    * When using a cost based planning approach this value defines the cost that is added for each step
    * taken. Setting this value to a high number will favor plans with less steps.
    */
   public default double getCostPerStep()
   {
      return 0.15;
   }

   /**
    * Some node checkers will check if the body of the robot will move through a higher planar region
    * (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
    * ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
    * This should be set to be slightly above cinder block height (20.3cm) for Atlas.
    */
   public default double getBodyGroundClearance()
   {
      return 0.25;
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   public default double getMinXClearanceFromStance()
   {
      return 0.0;
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   public default double getMinYClearanceFromStance()
   {
      return 0.0;
   }
}
