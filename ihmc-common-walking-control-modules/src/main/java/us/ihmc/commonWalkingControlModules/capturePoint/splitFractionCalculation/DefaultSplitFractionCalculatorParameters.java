package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

public class DefaultSplitFractionCalculatorParameters implements SplitFractionCalculatorParametersReadOnly
{
   public boolean calculateSplitFractionsFromPositions()
   {
      return false;
   }

   public boolean calculateSplitFractionsFromArea()
   {
      return false;
   }

   /** {@inheritDoc} */
   public double getDefaultTransferSplitFraction()
   {
      return 0.5;
   }

   /** {@inheritDoc} */

   public double getStepHeightForLargeStepDown()
   {
      return 0.15;
   }

   /** {@inheritDoc} */
   public double getLargestStepDownHeight()
   {
      return 0.25;
   }


   /** {@inheritDoc} */
   public double getTransferSplitFractionAtFullDepth()
   {
      return 0.3;
   }

   /** {@inheritDoc} */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return 0.75;
   }

   /** {@inheritDoc} */
   public double getFractionLoadIfFootHasFullSupport()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   public double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   public double getFractionLoadIfOtherFootHasNoWidth()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   public double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return 0.5;
   }
}
