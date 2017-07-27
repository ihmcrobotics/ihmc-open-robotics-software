package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BipedalFootstepPlannerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

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
   private final YoDouble maximumStepReach = new YoDouble("maximumStepReach", registry);
   private final YoDouble minimumFootholdPercent = new YoDouble("minimumFootholdPercent", registry);

   private final YoDouble idealFootstepLength = new YoDouble("idealFootstepLength", registry);
   private final YoDouble idealFootstepWidth = new YoDouble("idealFootstepWidth", registry);

   /**
    * Maximum vertical distance between consecutive footsteps
    *
    * <p>
    * A candidate footstep will be rejected if its z-value is greater than this quantity, when expressed its parent's
    * z-up sole frame.
    * </p>
    */
   private final YoDouble maximumStepZ = new YoDouble("maximumStepZ", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);

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
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);

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
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);

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
   private final YoDouble minimumStepLength = new YoDouble("minimumStepLength", registry);

   /**
    * Maximum step length and height when stepping forward and down.
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
   private final YoDouble maximumStepXWhenForwardAndDown = new YoDouble("maximumStepXWhenForwardAndDown", registry);
   private final YoDouble maximumStepZWhenForwardAndDown = new YoDouble("maximumStepZWhenForwardAndDown", registry);

   private final YoDouble wiggleInsideDelta = new YoDouble("wiggleInsideDelta", registry);
   private final YoBoolean rejectIfCannotFullyWiggleInside = new YoBoolean("rejectIfCannotFullyWiggleInside", registry);
   private final YoBoolean wiggleIntoConvexHullOfPlanarRegions = new YoBoolean("WiggleIntoConvexHullOfPlanarRegions", registry);

   private final YoDouble maximumXYWiggleDistance = new YoDouble("maximumXYWiggleDistance", registry);
   private final YoDouble maximumYawWiggle = new YoDouble("maximumYawWiggle", registry);

   private final YoDouble cliffHeightToShiftAwayFrom = new YoDouble("cliffHeightToShiftAwayFrom", registry);
   private final YoDouble minimumDistanceFromCliffBottoms = new YoDouble("minimumDistanceFromCliffBottoms", registry);

   private double minimumSurfaceNormalZ = 0.7;
   private double maximumZPenetrationOnVRegions = 0.008;

   public BipedalFootstepPlannerParameters(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      maximumXYWiggleDistance.set(0.1);
      maximumYawWiggle.set(0.1);
      rejectIfCannotFullyWiggleInside.set(false);
      wiggleIntoConvexHullOfPlanarRegions.set(true);
   }

   public void setRejectIfCannotFullyWiggleInside(boolean rejectIfCannotFullyWiggleInside)
   {
      this.rejectIfCannotFullyWiggleInside.set(rejectIfCannotFullyWiggleInside);
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach.set(maximumStepReach);
   }

   public void setMaximumStepZ(double maximumStepZ)
   {
      this.maximumStepZ.set(maximumStepZ);
   }

   public void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      this.maximumStepXWhenForwardAndDown.set(maximumStepXWhenForwardAndDown);
   }

   public void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      this.maximumStepZWhenForwardAndDown.set(maximumStepZWhenForwardAndDown);
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw.set(maximumStepYaw);
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth.set(minimumStepWidth);
   }

   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth.set(maximumStepWidth);
   }
   
   public void setMinimumStepLength(double minimumStepLength)
   {
      this.minimumStepLength.set(minimumStepLength);
   }

   public void setMinimumFootholdPercent(double minimumFootholdPercent)
   {
      this.minimumFootholdPercent.set(minimumFootholdPercent);
   }

   public void setIdealFootstep(double idealFootstepLength, double idealFootstepWidth)
   {
      this.idealFootstepLength.set(idealFootstepLength);
      this.idealFootstepWidth.set(idealFootstepWidth);
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta.set(wiggleInsideDelta);
   }

   public void setMaximumXYWiggleDistance(double maximumXYWiggleDistance)
   {
      this.maximumXYWiggleDistance.set(maximumXYWiggleDistance);
   }

   public void setMaximumYawWiggle(double maximumYawWiggle)
   {
      this.maximumYawWiggle.set(maximumYawWiggle);
   }

   public void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggleIntoConvexHullOfPlanarRegions)
   {
      this.wiggleIntoConvexHullOfPlanarRegions.set(wiggleIntoConvexHullOfPlanarRegions);      
   }

   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth.getDoubleValue();
   }

   public double getIdealFootstepLength()
   {
      return idealFootstepLength.getDoubleValue();
   }

   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta.getDoubleValue();
   }

   public double getMaximumStepReach()
   {
      return maximumStepReach.getDoubleValue();
   }

   public double getMaximumStepYaw()
   {
      return maximumStepYaw.getDoubleValue();
   }

   public double getMinimumStepWidth()
   {
      return minimumStepWidth.getDoubleValue();
   }

   public double getMinimumStepLength()
   {
      return minimumStepLength.getDoubleValue();
   }

   public double getMaximumStepXWhenForwardAndDown()
   {
      return maximumStepXWhenForwardAndDown.getDoubleValue();
   }
   
   public double getMaximumStepZWhenForwardAndDown()
   {
      return maximumStepZWhenForwardAndDown.getDoubleValue();
   }

   public double getMaximumStepZ()
   {
      return maximumStepZ.getDoubleValue();
   }

   public double getMinimumFootholdPercent()
   {
      return minimumFootholdPercent.getDoubleValue();
   }

   public double getMinimumSurfaceNormalZ()
   {
      return minimumSurfaceNormalZ;
   }

   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return wiggleIntoConvexHullOfPlanarRegions.getBooleanValue();
   }

   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return rejectIfCannotFullyWiggleInside.getBooleanValue();
   }

   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance.getDoubleValue();
   }

   public double getMaximumYawWiggle()
   {
      return maximumYawWiggle.getDoubleValue();
   }

   public double getMaximumZPenetrationOnVRegions()
   {
      return maximumZPenetrationOnVRegions;
   }

   public double getMaximumStepWidth()
   {
      return maximumStepWidth.getDoubleValue();
   }

   public double getCliffHeightToShiftAwayFrom()
   {
      return cliffHeightToShiftAwayFrom.getDoubleValue();
   }

   public void setCliffHeightToShiftAwayFrom(double cliffHeightToShiftAwayFrom)
   {
      this.cliffHeightToShiftAwayFrom.set(cliffHeightToShiftAwayFrom);
   }

   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms.getDoubleValue();
   }

   public void setMinimumDistanceFromCliffBottoms(double minimumDistanceFromCliffBottoms)
   {
      this.minimumDistanceFromCliffBottoms.set(minimumDistanceFromCliffBottoms);
   }


}
