package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

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
   private final DoubleParameter largestStepDownHeight;
   private final DoubleParameter transferSplitFractionAtFullDepth;
   private final DoubleParameter transferWeightDistributionAtFullDepth;
   private final DoubleParameter transferFinalWeightDistributionatFullDepth;
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
      largestStepDownHeight = new DoubleParameter("largestStepDownHeight", registry, defaultParameters.getLargestStepDownHeight());
      transferSplitFractionAtFullDepth = new DoubleParameter("transferSplitFractionAtFullDepth",
                                                             registry,
                                                             defaultParameters.getTransferSplitFractionAtFullDepth());
      transferWeightDistributionAtFullDepth = new DoubleParameter("transferWeightDistributionAtFullDepth",
                                                                  registry,
                                                                  defaultParameters.getTransferWeightDistributionAtFullDepth());
      transferFinalWeightDistributionatFullDepth = new DoubleParameter("transferFinalWeightDistributionAtFullDepth",
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

      registerVariableToSave(computeSplitFractionsFromArea);
      registerVariableToSave(computeSplitFractionsFromPositions);
      registerVariableToSave(defaultTransferSplitFraction);
      registerVariableToSave(stepHeightForLargeStepDown);
      registerVariableToSave(largestStepDownHeight);
      registerVariableToSave(transferSplitFractionAtFullDepth);
      registerVariableToSave(transferWeightDistributionAtFullDepth);
      registerVariableToSave(transferFinalWeightDistributionatFullDepth);
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
      return transferWeightDistributionAtFullDepth.getValue();
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
