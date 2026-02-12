package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSplitFractionCalculatorParameters extends YoSaveableModuleState
{
   public YoSplitFractionCalculatorParameters(SplitFractionCalculatorParameters defaultParameters, YoRegistry registry)
   {
      BooleanParameter computeSplitFractionsFromPositions = new BooleanParameter("computeSplitFractionsFromPositions",
                                                                                 registry,
                                                                                 defaultParameters.calculateSplitFractionsFromPositions());
      BooleanParameter computeSplitFractionsFromArea = new BooleanParameter("computeSplitFractionsFromArea",
                                                                            registry,
                                                                            defaultParameters.calculateSplitFractionsFromArea());
      DoubleParameter defaultTransferSplitFraction = new DoubleParameter("defaultTransferSplitFraction",
                                                                         registry,
                                                                         defaultParameters.getDefaultTransferSplitFraction());
      DoubleParameter stepHeightForLargeStepDown = new DoubleParameter("stepHeightForLargeStepDown",
                                                                       registry,
                                                                       defaultParameters.getStepHeightForLargeStepDown());
      DoubleParameter stepHeightForLargeStepUp = new DoubleParameter("stepHeightForLargeStepUp", registry, defaultParameters.getStepHeightForLargeStepUp());
      DoubleParameter largestStepDownHeight = new DoubleParameter("largestStepDownHeight", registry, defaultParameters.getLargestStepDownHeight());
      DoubleParameter largestStepUpHeight = new DoubleParameter("largestStepUpHeight", registry, defaultParameters.getLargestStepUpHeight());
      DoubleParameter transferSplitFractionAtFullDepth = new DoubleParameter("transferSplitFractionAtFullDepth",
                                                                             registry,
                                                                             defaultParameters.getTransferSplitFractionAtFullDepth());
      DoubleParameter transferSplitFractionForStepUpAtFullDepth = new DoubleParameter("transferSplitFractionForStepUpAtFullDepth",
                                                                                      registry,
                                                                                      defaultParameters.getTransferSplitFractionForStepUpAtFullDepth());
      DoubleParameter transferWeightDistributionAtFullDepth = new DoubleParameter("transferWeightDistributionAtFullDepth",
                                                                                  registry,
                                                                                  defaultParameters.getTransferWeightDistributionAtFullDepth());
      DoubleParameter transferWeightDistributionForStepUpAtFullDepth = new DoubleParameter("transferWeightDistributionForStepUpAtFullDepth",
                                                                                           registry,
                                                                                           defaultParameters.getTransferWeightDistributionForStepUpAtFullDepth());
      DoubleParameter transferFinalWeightDistributionAtFullDepth = new DoubleParameter("transferFinalWeightDistributionAtFullDepth",
                                                                                       registry,
                                                                                       defaultParameters.getTransferFinalWeightDistributionAtFullDepth());
      DoubleParameter transferFinalWeightDistributionForStepUpAtFullDepth = new DoubleParameter("transferFinalWeightDistributionForStepUpAtFullDepth",
                                                                                                registry,
                                                                                                defaultParameters.getTransferFinalWeightDistributionForStepUpAtFullDepth());
      DoubleParameter fractionLoadIfFootHasFullSupport = new DoubleParameter("fractionLoadIfFootHasFullSupport",
                                                                             registry,
                                                                             defaultParameters.getFractionLoadIfFootHasFullSupport());
      DoubleParameter fractionTimeOnFootIfFootHasFullSupport = new DoubleParameter("fractionTimeOnFootIfFootHasFullSupport",
                                                                                   registry,
                                                                                   defaultParameters.getFractionTimeOnFootIfFootHasFullSupport());
      DoubleParameter fractionLoadIfOtherFootHasNoWidth = new DoubleParameter("fractionLoadIfOtherFootHasNoWidth",
                                                                              registry,
                                                                              defaultParameters.getFractionLoadIfOtherFootHasNoWidth());
      DoubleParameter fractionTimeOnFootIfOtherFootHasNoWidth = new DoubleParameter("fractionTimeOnFootIfOtherFootHasNoWidth",
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
}
