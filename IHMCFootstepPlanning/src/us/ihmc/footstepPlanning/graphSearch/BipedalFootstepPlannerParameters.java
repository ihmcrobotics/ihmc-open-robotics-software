package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BipedalFootstepPlannerParameters
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
