package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;

public class AtlasICPSplitFractionCalculatorParameters implements SplitFractionCalculatorParametersReadOnly
{
   public boolean calculateSplitFractionsFromPositions()
   {
      return true;
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
      return 0.1;
   }

   /** {@inheritDoc} */
   public double getStepHeightForLargeStepUp()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   public double getLargestStepDownHeight()
   {
      return 0.15;
   }

   /** {@inheritDoc} */
   public double getLargestStepUpHeight()
   {
      return 0.25;
   }

   /** {@inheritDoc} */
   public double getTransferSplitFractionAtFullDepth()
   {
      return 0.3;
   }

   /** {@inheritDoc} */
   public double getTransferSplitFractionForStepUpAtFullDepth()
   {
      return 0.6;
   }

   /** {@inheritDoc} */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return 0.65;
   }

   /** {@inheritDoc} */
   public double getTransferWeightDistributionForStepUpAtFullDepth()
   {
      return 0.25;
   }

   /** {@inheritDoc} */
   public double getTransferFinalWeightDistributionAtFullDepth()
   {
      return 0.8;
   }

   /** {@inheritDoc} */
   public double getTransferFinalWeightDistributionForStepUpAtFullDepth()
   {
      return 0.35;
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
