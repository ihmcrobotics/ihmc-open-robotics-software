package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;

public class AtlasStepAdjustmentParameters extends StepAdjustmentParameters
{
   private static final boolean useStepAdjustment = true;

   /** {@inheritDoc} */
   @Override
   public boolean allowStepAdjustment()
   {
      return useStepAdjustment;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdjustmentDeadband()
   {
      return 0.02;
   }

   @Override
   public double getMinimumTimeForStepAdjustment()
   {
      return -0.2;
   }
}
