package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSplitFractionCalculatorParameters implements SplitFractionCalculatorParametersReadOnly
{
   private final BooleanProvider computeSplitFractionsFromPositions;
   private final BooleanProvider computeSplitFractionsFromArea;
   private final DoubleProvider defaultTransferSplitFraction;
   private final DoubleProvider stepHeightForLargeStepDown;
   private final DoubleProvider largestStepDownHeight;
   private final DoubleProvider transferSplitFractionAtFullDepth;
   private final DoubleProvider transferWeightDistributionatFullDepth;
   private final DoubleProvider transferFinalWeightDistributionatFullDepth;
   private final DoubleProvider fractionLoadIfFootHasFullSupport;
   private final DoubleProvider fractionTimeOnFootIfFootHasFullSupport;
   private final DoubleProvider fractionLoadIfOtherFootHasNoWidth;
   private final DoubleProvider fractionTimeOnFootIfOtherFootHasNoWidth;

   public YoSplitFractionCalculatorParameters(SplitFractionCalculatorParametersReadOnly defaultParameters, YoRegistry registry)
   {
      computeSplitFractionsFromPositions = new BooleanParameter("computeSplitFractionsFromPositions",
                                                                registry,
                                                                defaultParameters.calculateSplitFractionsFromPositions());
      computeSplitFractionsFromArea = new BooleanParameter("computeSplitFractionsFromArea", registry, defaultParameters.calculateSplitFractionsFromArea());
      defaultTransferSplitFraction = new DoubleParameter("defaultTransferSplitFraction", registry, defaultParameters.getDefaultTransferSplitFraction());
      stepHeightForLargeStepDown = new DoubleParameter("stepHeightForLargeStepDown", registry, defaultParameters.getStepHeightForLargeStepDown());
      largestStepDownHeight = new DoubleParameter("largestStepDownHeight", registry, defaultParameters.getLargestStepDownHeight());
      transferSplitFractionAtFullDepth = new DoubleParameter("transferSplitFractionAtFullDepth",
                                                             registry,
                                                             defaultParameters.getTransferSplitFractionAtFullDepth());
      transferWeightDistributionatFullDepth = new DoubleParameter("transferWeightDistributionatFullDepth",
                                                                  registry,
                                                                  defaultParameters.getTransferWeightDistributionAtFullDepth());
      transferFinalWeightDistributionatFullDepth = new DoubleParameter("transferFinalWeightDistributionatFullDepth",
                                                                       registry,
                                                                       defaultParameters.getTransferFinalWeightDistributionAtFullDepth());
      fractionLoadIfFootHasFullSupport = new DoubleParameter("fractionLoadIfFootHasFullSupport",
                                                             registry,
                                                             defaultParameters.getFractionLoadIfFootHasFullSupport());
      fractionTimeOnFootIfFootHasFullSupport = new DoubleParameter("fractionTimeOnFootIfFootHasFullSupport",
                                                                   registry,
                                                                   defaultParameters.getFractionTimeOnFootIfFootHasFullSupport());
      fractionLoadIfOtherFootHasNoWidth = new DoubleParameter("fractionLoadIfOtherFootHasNoWidth",
                                                              registry,
                                                              defaultParameters.getFractionLoadIfOtherFootHasNoWidth());
      fractionTimeOnFootIfOtherFootHasNoWidth = new DoubleParameter("fractionTimeOnFootIfOtherFootHasNoWidth",
                                                                    registry,
                                                                    defaultParameters.getFractionTimeOnFootIfOtherFootHasNoWidth());
   }

   public boolean calculateSplitFractionsFromPositions()
   {
      return computeSplitFractionsFromPositions.getValue();
   }

   public boolean calculateSplitFractionsFromArea()
   {
      return computeSplitFractionsFromArea.getValue();
   }

   /** {@inheritDoc} */
   public double getDefaultTransferSplitFraction()
   {
      return defaultTransferSplitFraction.getValue();
   }

   /** {@inheritDoc} */
   public double getStepHeightForLargeStepDown()
   {
      return stepHeightForLargeStepDown.getValue();
   }

   /** {@inheritDoc} */
   public double getLargestStepDownHeight()
   {
      return largestStepDownHeight.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferSplitFractionAtFullDepth()
   {
      return transferSplitFractionAtFullDepth.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return transferWeightDistributionatFullDepth.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferFinalWeightDistributionAtFullDepth()
   {
      return transferFinalWeightDistributionatFullDepth.getValue();
   }

   /** {@inheritDoc} */
   public double getFractionLoadIfFootHasFullSupport()
   {
      return fractionLoadIfFootHasFullSupport.getValue();
   }

   /** {@inheritDoc} */
   public double getFractionTimeOnFootIfFootHasFullSupport()
   {
      return fractionTimeOnFootIfFootHasFullSupport.getValue();
   }

   /** {@inheritDoc} */
   public double getFractionLoadIfOtherFootHasNoWidth()
   {
      return fractionLoadIfOtherFootHasNoWidth.getValue();
   }

   /** {@inheritDoc} */
   public double getFractionTimeOnFootIfOtherFootHasNoWidth()
   {
      return fractionTimeOnFootIfOtherFootHasNoWidth.getValue();
   }
}
