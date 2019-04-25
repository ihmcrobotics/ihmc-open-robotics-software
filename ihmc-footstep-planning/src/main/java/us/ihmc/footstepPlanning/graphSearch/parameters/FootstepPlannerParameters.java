package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.filters.BodyCollisionRegionFilter;
import us.ihmc.footstepPlanning.filters.SteppableRegionFilter;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface FootstepPlannerParameters
{
   /**
    * Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
    * to run slower.
    */
   default boolean checkForBodyBoxCollisions()
   {
      return false;
   }

   /**
    * Sets whether or not to perform the defined heuristic search policies.
    */
   default boolean performHeuristicSearchPolicies()
   {
      return true;
   }

   /**
    * Returns the ideal step width for walking on flat ground.
    */
   double getIdealFootstepWidth();

   /**
    * Returns the ideal step length for walking on flat ground.
    */
   double getIdealFootstepLength();

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
   default double getWiggleInsideDelta()
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
   double getMaximumStepReach();

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
   double getMaximumStepYaw();

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
   double getMinimumStepWidth();

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
      return -getMaximumStepReach();
   }

   /**
    * Minimum step yaw.
    */
   default double getMinimumStepYaw()
   {
      return 0.0;
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
      return getMaximumStepReach();
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
      return Double.POSITIVE_INFINITY;
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
   default double getMaximumStepZWhenForwardAndDown()
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
   double getMaximumStepZ();

   /**
    * Minimum percentage that a candidate footstep needs to overlap with its associated planar region in order to be accepted.
    * <p>
    * If this parameter is set to 1.0 only full footsteps are allowed. A value less then 1.0 will allow partial footholds.
    * </p>
    */
   default double getMinimumFootholdPercent()
   {
      return 0.9;
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
   default boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return true;
   }

   /**
    * If the planner uses footstep wiggling it attempts to move a candidate footstep inside its associated planar region.
    * This attempt is parametrized by {@link #getWiggleIntoConvexHullOfPlanarRegions()}, {@link #getWiggleInsideDelta},
    * {@link #getMaximumXYWiggleDistance}, and {@link #getMaximumYawWiggle}. If this transform cannot be found, the
    * candidate footstep will be rejected if this method returns {@code true}.
    */
   default boolean getRejectIfCannotFullyWiggleInside()
   {
      return false;
   }

   /**
    * When wiggling a candidate footstep into a planar region, this is the maximum distance xy-distance
    * distance the planner will use
    */
   default double getMaximumXYWiggleDistance()
   {
      return LatticeNode.gridSizeXY / 2.0;
   }

   /**
    * When wiggling a candidate footstep into a planar region, this is the maximum yaw
    * distance the planner will use
    */
   default double getMaximumYawWiggle()
   {
      return LatticeNode.gridSizeYaw / 2.0;
   }

   /**
    * When snapping a candidate footstep to a planar region, its possible that another planar region
    * intersects the footstep at a steep angle, i.e. a valley. If this intersecting planar region
    * is never more than maximumZPenetrationOnValleyRegions above the footstep, it won't be rejected,
    * otherwise it will.
    */
   default double getMaximumZPenetrationOnValleyRegions()
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
   double getMaximumStepWidth();

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
   default double getMinimumDistanceFromCliffBottoms()
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
   default boolean getReturnBestEffortPlan()
   {
      return false;
   }

   /**
    * When {@link #getReturnBestEffortPlan()} is true, the planner will return the best effort plan if the plan
    * contains at least this many footsteps.
    */
   default int getMinimumStepsForBestEffortPlan()
   {
      return 3;
   }

   /**
    * Some node checkers will check if the body of the robot will move through a higher planar region
    * (e.g. a wall) when going from one footstep to the next one. To avoid planar regions close to the
    * ground triggering this this parameter defines a ground clearance under which obstacles are allowed.
    * This should be set to be slightly above cinder block height (20.3cm) for Atlas.
    */
   default double getBodyGroundClearance()
   {
      return 0.25;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box height. Note that this box will go from {@code getBodyBoxBaseZ}
    * to {@code getBodyBoxBaseHeight + getBodyBoxHeight}
    */
   default double getBodyBoxHeight()
   {
      return 1.5;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box depth.
    */
   default double getBodyBoxDepth()
   {
      return 0.3;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box width.
    */
   default double getBodyBoxWidth()
   {
      return 0.7;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseX()
   {
      return 0.0;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseY()
   {
      return 0.0;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseZ()
   {
      return 0.25;
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   default double getMinXClearanceFromStance()
   {
      return 0.0;
   }

   /**
    * Parameter used inside the node expansion to avoid footsteps that would be on top of the stance foot.
    * Nodes are only added to the expanded list if they are outside the box around the stance foot defined by
    * this parameter.
    */
   default double getMinYClearanceFromStance()
   {
      return 0.0;
   }

   /**
    * Radius around the goal inside which the planner should start to turn to match the goal's orientation
    */
   default double getGoalTurnRadius()
   {
      return 1.0;
   }

   default FootstepPlannerCostParameters getCostParameters()
   {
      return new DefaultFootstepPlannerCostParameters();
   }

   default SteppableRegionFilter getSteppableRegionFilter()
   {
      return new SteppableRegionFilter()
      {
         private Vector3D vertical = new Vector3D(0.0, 0.0, 1.0);

         @Override
         public boolean isPlanarRegionSteppable(PlanarRegion query)
         {
            double angle = query.getNormal().angle(vertical);

            if (angle > getMinimumSurfaceInclineRadians() + 1e-5)
               return false;

            return true;
         }
      };
   }

   default BodyCollisionRegionFilter getBodyCollisionRegionFilter()
   {
      return new BodyCollisionRegionFilter()
      {
         @Override
         public boolean isPlanarRegionCollidable(PlanarRegion query, double groundHeight, double minHeight, double maxHeight)
         {
            if (query.getBoundingBox3dInWorld().getMaxZ() < minHeight + groundHeight)
               return false;

            return maxHeight + groundHeight > query.getBoundingBox3dInWorld().getMinZ();
         }
      };
   }

   /**
    * Parameters for setting swing trajectories from footstep poses. Will use default values if this returns null
    */
   default AdaptiveSwingParameters getAdaptiveSwingParameters()
   {
      return null;
   }
}
