package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoFootstepPlannerParameters implements FootstepPlannerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble maximumStepReach = new YoDouble("maximumStepReach", registry);
   private final YoDouble minimumFootholdPercent = new YoDouble("minimumFootholdPercent", registry);
   private final YoDouble idealFootstepLength = new YoDouble("idealFootstepLength", registry);
   private final YoDouble idealFootstepWidth = new YoDouble("idealFootstepWidth", registry);
   private final YoDouble maximumStepZ = new YoDouble("maximumStepZ", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);
   private final YoDouble minimumStepLength = new YoDouble("minimumStepLength", registry);
   private final YoDouble minimumStepYaw = new YoDouble("minimumStepYaw", registry);
   private final YoDouble maximumStepXWhenForwardAndDown = new YoDouble("maximumStepXWhenForwardAndDown", registry);
   private final YoDouble maximumStepZWhenForwardAndDown = new YoDouble("maximumStepZWhenForwardAndDown", registry);
   private final YoDouble wiggleInsideDelta = new YoDouble("wiggleInsideDelta", registry);
   private final YoBoolean rejectIfCannotFullyWiggleInside = new YoBoolean("rejectIfCannotFullyWiggleInside", registry);
   private final YoBoolean wiggleIntoConvexHullOfPlanarRegions = new YoBoolean("WiggleIntoConvexHullOfPlanarRegions", registry);
   private final YoDouble maximumXYWiggleDistance = new YoDouble("maximumXYWiggleDistance", registry);
   private final YoDouble maximumYawWiggle = new YoDouble("maximumYawWiggle", registry);
   private final YoDouble cliffHeightToShiftAwayFrom = new YoDouble("cliffHeightToShiftAwayFrom", registry);
   private final YoDouble minimumDistanceFromCliffBottoms = new YoDouble("minimumDistanceFromCliffBottoms", registry);
   private final YoDouble minimumSurfaceInclineRadians = new YoDouble("minimumSurfaceInclineRadians", registry);
   private final YoDouble maximumZPenetrationOnValleyRegions = new YoDouble("maximumZPenetrationOnValleyRegions", registry);
   private final YoDouble yawWeight = new YoDouble("yawWeight", registry);
   private final YoDouble costPerStep = new YoDouble("costPerStep", registry);
   private final YoDouble bodyGroundClearance = new YoDouble("bodyGroundClearance", registry);
   private final YoBoolean returnBestEffortPlan = new YoBoolean("returnBestEffortPlan", registry);
   private final YoInteger minimumStepForBestEffortPlan = new YoInteger("minimumStepForBestEffortPlan", registry);
   private final YoDouble minXClearanceFromStance = new YoDouble("minXClearanceFromStance", registry);
   private final YoDouble minYClearanceFromStance = new YoDouble("minYClearanceFromStance", registry);

   public YoFootstepPlannerParameters(YoVariableRegistry parentRegistry, FootstepPlannerParameters defaults)
   {
      parentRegistry.addChild(registry);
      set(defaults);
   }

   public void set(FootstepPlannerParameters defaults)
   {
      maximumStepReach.set(defaults.getMaximumStepReach());
      minimumFootholdPercent.set(defaults.getMinimumFootholdPercent());
      idealFootstepLength.set(defaults.getIdealFootstepLength());
      idealFootstepWidth.set(defaults.getIdealFootstepWidth());
      maximumStepZ.set(defaults.getMaximumStepZ());
      maximumStepYaw.set(defaults.getMaximumStepYaw());
      maximumStepWidth.set(defaults.getMaximumStepWidth());
      minimumStepWidth.set(defaults.getMinimumStepWidth());
      minimumStepLength.set(defaults.getMinimumStepLength());
      minimumStepYaw.set(defaults.getMinimumStepYaw());
      maximumStepXWhenForwardAndDown.set(defaults.getMaximumStepXWhenForwardAndDown());
      maximumStepZWhenForwardAndDown.set(defaults.getMaximumStepZWhenForwardAndDown());
      wiggleInsideDelta.set(defaults.getWiggleInsideDelta());
      rejectIfCannotFullyWiggleInside.set(defaults.getRejectIfCannotFullyWiggleInside());
      wiggleIntoConvexHullOfPlanarRegions.set(defaults.getWiggleIntoConvexHullOfPlanarRegions());
      maximumXYWiggleDistance.set(defaults.getMaximumXYWiggleDistance());
      maximumYawWiggle.set(defaults.getMaximumYawWiggle());
      cliffHeightToShiftAwayFrom.set(defaults.getCliffHeightToAvoid());
      minimumDistanceFromCliffBottoms.set(defaults.getMinimumDistanceFromCliffBottoms());
      minimumSurfaceInclineRadians.set(defaults.getMinimumSurfaceInclineRadians());
      maximumZPenetrationOnValleyRegions.set(defaults.getMaximumZPenetrationOnValleyRegions());
      yawWeight.set(defaults.getYawWeight());
      costPerStep.set(defaults.getCostPerStep());
      bodyGroundClearance.set(defaults.getBodyGroundClearance());
      returnBestEffortPlan.set(defaults.getReturnBestEffortPlan());
      minimumStepForBestEffortPlan.set(defaults.getMinimumStepsForBestEffortPlan());
      minXClearanceFromStance.set(defaults.getMinXClearanceFromStance());
      minYClearanceFromStance.set(defaults.getMinYClearanceFromStance());
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth.getDoubleValue();
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength.getDoubleValue();
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta.getDoubleValue();
   }

   @Override
   public double getMaximumStepReach()
   {
      return maximumStepReach.getDoubleValue();
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maximumStepYaw.getDoubleValue();
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minimumStepWidth.getDoubleValue();
   }

   @Override
   public double getMinimumStepLength()
   {
      return minimumStepLength.getDoubleValue();
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minimumStepYaw.getDoubleValue();
   }

   @Override
   public double getMaximumStepXWhenForwardAndDown()
   {
      return maximumStepXWhenForwardAndDown.getDoubleValue();
   }

   @Override
   public double getMaximumStepZWhenForwardAndDown()
   {
      return maximumStepZWhenForwardAndDown.getDoubleValue();
   }

   @Override
   public double getMaximumStepZ()
   {
      return maximumStepZ.getDoubleValue();
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minimumFootholdPercent.getDoubleValue();
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians.getDoubleValue();
   }

   @Override
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return wiggleIntoConvexHullOfPlanarRegions.getBooleanValue();
   }

   @Override
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return rejectIfCannotFullyWiggleInside.getBooleanValue();
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance.getDoubleValue();
   }

   @Override
   public double getMaximumYawWiggle()
   {
      return maximumYawWiggle.getDoubleValue();
   }

   @Override
   public double getMaximumZPenetrationOnValleyRegions()
   {
      return maximumZPenetrationOnValleyRegions.getDoubleValue();
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maximumStepWidth.getDoubleValue();
   }

   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToShiftAwayFrom.getDoubleValue();
   }

   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms.getDoubleValue();
   }

   @Override
   public double getYawWeight()
   {
      return yawWeight.getDoubleValue();
   }

   @Override
   public double getCostPerStep()
   {
      return costPerStep.getDoubleValue();
   }

   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance.getDoubleValue();
   }

   @Override
   public boolean getReturnBestEffortPlan()
   {
      return returnBestEffortPlan.getBooleanValue();
   }

   @Override
   public int getMinimumStepsForBestEffortPlan()
   {
      return minimumStepForBestEffortPlan.getIntegerValue();
   }

   @Override
   public double getMinXClearanceFromStance()
   {
      return minXClearanceFromStance.getDoubleValue();
   }

   @Override
   public double getMinYClearanceFromStance()
   {
      return minYClearanceFromStance.getDoubleValue();
   }
}
