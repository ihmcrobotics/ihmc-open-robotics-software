package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

public class DefaultSplitFractionCalculatorParameters implements SplitFractionCalculatorParametersBasics
{
   private boolean calculateSplitFractionsFromPositions = true;
   private boolean calculateSplitFractionsFromArea = false;

   private double defaultTransferSplitFraction = 0.5;
   private double stepHeightForLargeStepDown = 0.1;
   private double stepHeightForLargeStepUp = 0.5;
   private double largestStepDownHeight = 0.15;
   private double largestStepUpHeight = 0.25;

   private double transferSplitFractionAtFullDepth = 0.3;
   private double transferSplitFractionForStepUpAtFullDepth = 0.6;
   private double transferWeightDistributionAtFullDepth = 0.65;
   private double transferWeightDistributionForStepUpAtFullDepth = 0.25;
   private double transferFinalWeightDistributionAtFullDepth = 0.8;
   private double transferFinalWeightDistributionForStepUpAtFullDepth = 0.35;

   private double fractionLoadIfFootHasFullSupport = 0.5;
   private double fractionTimeOnFootIfFootHasFullSupport = 0.5;
   private double fractionLoadIfOtherFootHasNoWidth = 0.5;
   private double fractionTimeOnFootIfOtherFootHasNoWidth = 0.5;

   @Override
   public boolean calculateSplitFractionsFromPositions()
   {
      return calculateSplitFractionsFromPositions;
   }

   @Override
   public void setCalculateSplitFractionsFromPositions(boolean calculateSplitFractionsFromPositions)
   {
      this.calculateSplitFractionsFromPositions = calculateSplitFractionsFromPositions;
   }

   @Override
   public boolean calculateSplitFractionsFromArea()
   {
      return calculateSplitFractionsFromArea;
   }

   @Override
   public void setCalculateSplitFractionsFromArea(boolean calculateSplitFractionsFromArea)
   {
      this.calculateSplitFractionsFromArea = calculateSplitFractionsFromArea;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultTransferSplitFraction()
   {
      return defaultTransferSplitFraction;
   }

   @Override
   public void setDefaultTransferSplitFraction(double splitFraction)
   {
      this.defaultTransferSplitFraction = splitFraction;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepHeightForLargeStepDown()
   {
      return stepHeightForLargeStepDown;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepHeightForLargeStepUp() {
      return stepHeightForLargeStepUp;
   }

   @Override
   public void setStepHeightForLargeStepDown(double height)
   {
      this.stepHeightForLargeStepDown = height;
   }

   /** {@inheritDoc} */
   @Override
   public double getLargestStepDownHeight()
   {
      return largestStepDownHeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getLargestStepUpHeight() {
      return largestStepUpHeight;
   }

   @Override
   public void setLargestStepDownHeight(double height)
   {
      this.largestStepDownHeight = height;
   }

   @Override
   public void setLargestStepUpHeight(double height)
   {
      this.largestStepUpHeight = height;
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferSplitFractionAtFullDepth()
   {
      return transferSplitFractionAtFullDepth;
   }

   @Override
   public void setTransferSplitFractionAtFullDepth(double splitFraction)
   {
      this.transferSplitFractionAtFullDepth = splitFraction;
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferSplitFractionForStepUpAtFullDepth()
   {
      return transferSplitFractionForStepUpAtFullDepth;
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferWeightDistributionAtFullDepth()
   {
      return transferWeightDistributionAtFullDepth;
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferWeightDistributionForStepUpAtFullDepth()
   {
      return transferWeightDistributionForStepUpAtFullDepth;
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferFinalWeightDistributionAtFullDepth()
   {
      return transferFinalWeightDistributionAtFullDepth;
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferFinalWeightDistributionForStepUpAtFullDepth()
   {
      return transferFinalWeightDistributionForStepUpAtFullDepth;
   }

   @Override
   public void setTransferWeightDistributionAtFullDepth(double weightDistribution)
   {
      this.transferWeightDistributionAtFullDepth = weightDistribution;
   }

   /** {@inheritDoc} */
   @Override
   public double getFractionLoadIfFootHasFullSupport()
   {
      return fractionLoadIfFootHasFullSupport;
   }

   @Override
   public void setFractionLoadIfFootHasFullSupport(double fractionLoad)
   {
      this.fractionLoadIfFootHasFullSupport = fractionLoad;
   }

   /** {@inheritDoc} */
   @Override
   public double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return fractionTimeOnFootIfFootHasFullSupport;
   }

   @Override
   public void setFractionTimeOnFootIfFootHasFullSupport(double fractionTime)
   {
      this.fractionTimeOnFootIfFootHasFullSupport = fractionTime;
   }

   /** {@inheritDoc} */
   @Override
   public double getFractionLoadIfOtherFootHasNoWidth()
   {
      return fractionLoadIfOtherFootHasNoWidth;
   }

   @Override
   public void setFractionLoadIfOtherFootHasNoWidth(double fractionLoad)
   {
      this.fractionLoadIfOtherFootHasNoWidth = fractionLoad;
   }

   /** {@inheritDoc} */
   @Override
   public double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return fractionTimeOnFootIfOtherFootHasNoWidth;
   }

   @Override
   public void setFractionTimeOnFootIfOtherFootHasNoWidth(double fractionTime)
   {
      this.fractionTimeOnFootIfOtherFootHasNoWidth = fractionTime;
   }
}
