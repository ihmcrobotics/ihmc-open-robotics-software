package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class BipedalFootstepPlannerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable maximumStepReach = new DoubleYoVariable("maximumStepReach", registry);
   private final DoubleYoVariable minimumFootholdPercent = new DoubleYoVariable("minimumFootholdPercent", registry);

   private final DoubleYoVariable idealFootstepLength = new DoubleYoVariable("idealFootstepLength", registry);
   private final DoubleYoVariable idealFootstepWidth = new DoubleYoVariable("idealFootstepWidth", registry);

   private final DoubleYoVariable maximumStepZ = new DoubleYoVariable("maximumStepZ", registry);
   private final DoubleYoVariable maximumStepYaw = new DoubleYoVariable("maximumStepYaw", registry);
   private final DoubleYoVariable minimumStepWidth = new DoubleYoVariable("minimumStepWidth", registry);

   private final DoubleYoVariable maximumStepXWhenForwardAndDown = new DoubleYoVariable("maximumStepXWhenForwardAndDown", registry);
   private final DoubleYoVariable maximumStepZWhenForwardAndDown = new DoubleYoVariable("maximumStepZWhenForwardAndDown", registry);

   private final DoubleYoVariable wiggleInsideDelta = new DoubleYoVariable("wiggleInsideDelta", registry);
   private final BooleanYoVariable rejectIfCannotFullyWiggleInside = new BooleanYoVariable("rejectIfCannotFullyWiggleInside", registry);
   private final BooleanYoVariable wiggleIntoConvexHullOfPlanarRegions = new BooleanYoVariable("WiggleIntoConvexHullOfPlanarRegions", registry);

   private final DoubleYoVariable maximumXYWiggleDistance = new DoubleYoVariable("maximumXYWiggleDistance", registry);
   private final DoubleYoVariable maximumYawWiggle = new DoubleYoVariable("maximumYawWiggle", registry);

   private double minimumSurfaceNormalZ = 0.7;
   private double maximumZPenetrationOnVRegions = 0.008;

   BipedalFootstepPlannerParameters(YoVariableRegistry parentRegistry)
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
}
