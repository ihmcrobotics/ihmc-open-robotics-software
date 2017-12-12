package us.ihmc.commonWalkingControlModules.configurations;

public abstract class AbstractICPPlannerParameters extends ICPWithTimeFreezingPlannerParameters
{
   protected final double modelScale;

   protected AbstractICPPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;
   }

   /**
    * <p>
    * {@inheritDoc}
    * </p>
    * <p>
    * The values 3 and 4 seem to be good.
    * </p>
    */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   @Override
   /** {@inheritDoc} */
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return modelScale * 0.025;
   }

   @Override
   /** {@inheritDoc} */
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return modelScale * 0.03;
   }

   @Override
   /** {@inheritDoc} */
   public boolean getDoTimeFreezing()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   @Override
   /** {@inheritDoc} */
   public double getVelocityDecayDurationWhenDone()
   {
      return Double.NaN;
   }

   @Override
   /** {@inheritDoc} */
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   @Override
   /** {@inheritDoc} */
   public boolean putExitCoPOnToes()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public boolean useExitCoPOnToesForSteppingDown()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return modelScale * 0.15;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepHeightThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return modelScale * 0.10;
   }

   @Override
   /** {@inheritDoc} */
   public double getCoPSafeDistanceAwayFromToesWhenSteppingDown()
   {
      return modelScale * 0.0;
   }

   @Override
   /** {@inheritDoc} */
   public double getExitCoPForwardSafetyMarginOnToes()
   {
      return modelScale * 1.6e-2;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepLengthThresholdForExitCoPOnToes()
   {
      return modelScale * 0.15;
   }
}