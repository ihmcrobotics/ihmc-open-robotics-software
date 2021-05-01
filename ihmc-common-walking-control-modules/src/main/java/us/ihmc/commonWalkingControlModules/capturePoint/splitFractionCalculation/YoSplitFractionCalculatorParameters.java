package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSplitFractionCalculatorParameters extends YoSaveableModuleState implements SplitFractionCalculatorParametersReadOnly
{
   private final BooleanParameter computeSplitFractionsFromPositions;
   private final BooleanParameter computeSplitFractionsFromArea;
   private final DoubleParameter defaultTransferSplitFraction;
   private final DoubleParameter stepHeightForLargeStepDown;
   private final DoubleParameter stepHeightForLargeStepUp;
   private final DoubleParameter largestStepDownHeight;
   private final DoubleParameter largestStepUpHeight;
   private final DoubleParameter transferSplitFractionAtFullDepth;
   private final DoubleParameter transferSplitFractionForStepUpAtFullDepth;
   private final DoubleParameter transferWeightDistributionAtFullDepth;
   private final DoubleParameter transferWeightDistributionForStepUpAtFullDepth;
   private final DoubleParameter transferFinalWeightDistributionAtFullDepth;
   private final DoubleParameter transferFinalWeightDistributionForStepUpAtFullDepth;
   private final DoubleParameter fractionLoadIfFootHasFullSupport;
   private final DoubleParameter fractionTimeOnFootIfFootHasFullSupport;
   private final DoubleParameter fractionLoadIfOtherFootHasNoWidth;
   private final DoubleParameter fractionTimeOnFootIfOtherFootHasNoWidth;

   public YoSplitFractionCalculatorParameters(SplitFractionCalculatorParametersReadOnly defaultParameters, YoRegistry registry)
   {
      computeSplitFractionsFromPositions = new BooleanParameter("computeSplitFractionsFromPositions",
                                                                registry,
                                                                defaultParameters.calculateSplitFractionsFromPositions());
      computeSplitFractionsFromArea = new BooleanParameter("computeSplitFractionsFromArea", registry, defaultParameters.calculateSplitFractionsFromArea());
      defaultTransferSplitFraction = new DoubleParameter("defaultTransferSplitFraction", registry, defaultParameters.getDefaultTransferSplitFraction());
      stepHeightForLargeStepDown = new DoubleParameter("stepHeightForLargeStepDown", registry, defaultParameters.getStepHeightForLargeStepDown());
      stepHeightForLargeStepUp = new DoubleParameter("stepHeightForLargeStepUp", registry, defaultParameters.getStepHeightForLargeStepUp());
      largestStepDownHeight = new DoubleParameter("largestStepDownHeight", registry, defaultParameters.getLargestStepDownHeight());
      largestStepUpHeight = new DoubleParameter("largestStepUpHeight", registry, defaultParameters.getLargestStepUpHeight());
      transferSplitFractionAtFullDepth = new DoubleParameter("transferSplitFractionAtFullDepth",
                                                             registry,
                                                             defaultParameters.getTransferSplitFractionAtFullDepth());
      transferSplitFractionForStepUpAtFullDepth = new DoubleParameter("transferSplitFractionForStepUpAtFullDepth",
                                                              registry,
                                                              defaultParameters.getTransferSplitFractionForStepUpAtFullDepth());
      transferWeightDistributionAtFullDepth = new DoubleParameter("transferWeightDistributionAtFullDepth",
                                                                  registry,
                                                                  defaultParameters.getTransferWeightDistributionAtFullDepth());
      transferWeightDistributionForStepUpAtFullDepth = new DoubleParameter("transferWeightDistributionForStepUpAtFullDepth",
                                                              registry,
                                                              defaultParameters.getTransferWeightDistributionForStepUpAtFullDepth());
      transferFinalWeightDistributionAtFullDepth = new DoubleParameter("transferFinalWeightDistributionAtFullDepth",
                                                                       registry,
                                                                       defaultParameters.getTransferFinalWeightDistributionAtFullDepth());
      transferFinalWeightDistributionForStepUpAtFullDepth = new DoubleParameter("transferFinalWeightDistributionForStepUpAtFullDepth",
                                                              registry,
                                                              defaultParameters.getTransferFinalWeightDistributionForStepUpAtFullDepth());
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

      registerVariableToSave(computeSplitFractionsFromArea);
      registerVariableToSave(computeSplitFractionsFromPositions);
      registerVariableToSave(defaultTransferSplitFraction);
      registerVariableToSave(stepHeightForLargeStepDown);
      registerVariableToSave(stepHeightForLargeStepUp);
      registerVariableToSave(largestStepDownHeight);
      registerVariableToSave(largestStepUpHeight);
      registerVariableToSave(transferSplitFractionAtFullDepth);
      registerVariableToSave(transferSplitFractionForStepUpAtFullDepth);
      registerVariableToSave(transferWeightDistributionAtFullDepth);
      registerVariableToSave(transferWeightDistributionForStepUpAtFullDepth);
      registerVariableToSave(transferFinalWeightDistributionAtFullDepth);
      registerVariableToSave(transferFinalWeightDistributionForStepUpAtFullDepth);
      registerVariableToSave(fractionLoadIfFootHasFullSupport);
      registerVariableToSave(fractionTimeOnFootIfFootHasFullSupport);
      registerVariableToSave(fractionLoadIfOtherFootHasNoWidth);
      registerVariableToSave(fractionTimeOnFootIfOtherFootHasNoWidth);

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

   @Override
   public double getStepHeightForLargeStepUp()
   {
      return stepHeightForLargeStepUp.getValue();
   }

   /** {@inheritDoc} */
   public double getLargestStepDownHeight()
   {
      return largestStepDownHeight.getValue();
   }

   @Override
   public double getLargestStepUpHeight()
   {
      return largestStepUpHeight.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferSplitFractionAtFullDepth()
   {
      return transferSplitFractionAtFullDepth.getValue();
   }

   @Override
   public double getTransferSplitFractionForStepUpAtFullDepth()
   {
      return transferSplitFractionForStepUpAtFullDepth.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferWeightDistributionAtFullDepth()
   {
      return transferWeightDistributionAtFullDepth.getValue();
   }

   @Override
   public double getTransferWeightDistributionForStepUpAtFullDepth()
   {
      return transferWeightDistributionForStepUpAtFullDepth.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferFinalWeightDistributionAtFullDepth()
   {
      return transferFinalWeightDistributionAtFullDepth.getValue();
   }

   /** {@inheritDoc} */
   public double getTransferFinalWeightDistributionForStepUpAtFullDepth()
   {
      return transferFinalWeightDistributionForStepUpAtFullDepth.getValue();
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
