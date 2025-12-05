package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;

public class OpenAlexanderStepAdjustmentParameters extends StepAdjustmentParameters
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
      return 0.005;
   }

   @Override
   public double getMinimumTimeForStepAdjustment()
   {
      return -0.2;
   }
}
