package us.ihmc.footstepPlanning.icp;

import controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface SplitFractionCalculatorParametersBasics extends SplitFractionCalculatorParametersReadOnly, StoredPropertySetBasics
{
   default void set(SplitFractionCalculatorParametersReadOnly parameters)
   {
      setAll(parameters.getAll());
   }

   default void setDefaultTransferSplitFraction(double defaultTransferSplitFraction)
   {
      set(SplitFractionCalculatorParameterKeys.defaultTransferSplitFraction, defaultTransferSplitFraction);
   }

   default void setStepHeightForLargeStepDown(double height)
   {
      set(SplitFractionCalculatorParameterKeys.stepHeightForLargeStepDown, height);
   }

   default void setLargestStepDownHeight(double height)
   {
      set(SplitFractionCalculatorParameterKeys.largestStepDownHeight, height);
   }

   default void setTransferSplitFractionAtFullDepth(double splitFraction)
   {
      set(SplitFractionCalculatorParameterKeys.transferSplitFractionAtFullDepth, splitFraction);
   }

   default void setTransferWeightDistributionAtFullDepth(double weightDistribution)
   {
      set(SplitFractionCalculatorParameterKeys.transferWeightDistributionAtFullDepth, weightDistribution);
   }

   default void setFractionLoadIfFootHasFullSupport(double fraction)
   {
      set(SplitFractionCalculatorParameterKeys.fractionLoadIfFootHasFullSupport, fraction);
   }

   default void setFractionTimeOnFootIfFootHasFullSupport(double fraction)
   {
      set(SplitFractionCalculatorParameterKeys.fractionTimeOnFootIfFootHasFullSupport, fraction);
   }

   default void setFractionLoadIfOtherFootHasNoWidth(double fraction)
   {
      set(SplitFractionCalculatorParameterKeys.fractionLoadIfOtherFootHasNoWidth, fraction);
   }

   default void setFractionTimeOnFootIfOtherFootHasNoWidth(double fraction)
   {
      set(SplitFractionCalculatorParameterKeys.fractionTimeOnFootIfOtherFootHasNoWidth, fraction);
   }

   default void set(SplitFractionCalculatorParametersPacket packet)
   {
      if (packet.getDefaultTransferSplitFraction() != -1.0)
         setDefaultTransferSplitFraction(packet.getDefaultTransferSplitFraction());
      if (packet.getStepHeightForLargeStepDown() != -1.0)
         setStepHeightForLargeStepDown(packet.getStepHeightForLargeStepDown());
      if (packet.getLargestStepDownHeight() != -1.0)
         setLargestStepDownHeight(packet.getLargestStepDownHeight());
      if (packet.getTransferSplitFractionAtFullDepth() != -1.0)
         setTransferSplitFractionAtFullDepth(packet.getTransferSplitFractionAtFullDepth());
      if (packet.getTransferWeightDistributionAtFullDepth() != -1.0)
         setTransferWeightDistributionAtFullDepth(packet.getTransferWeightDistributionAtFullDepth());

      if (packet.getFractionLoadIfFootHasFullSupport() != -1.0)
         setFractionLoadIfFootHasFullSupport(packet.getFractionLoadIfFootHasFullSupport());
      if (packet.getFractionTimeOnFootIfFootHasFullSupport() != -1.0)
         setFractionTimeOnFootIfFootHasFullSupport(packet.getFractionTimeOnFootIfFootHasFullSupport());
      if (packet.getFractionLoadIfOtherFootHasNoWidth() != -1.0)
         setFractionLoadIfOtherFootHasNoWidth(packet.getFractionLoadIfOtherFootHasNoWidth());
      if (packet.getFractionTimeOnFootIfOtherFootHasNoWidth() != -1.0)
         setFractionTimeOnFootIfOtherFootHasNoWidth(packet.getFractionTimeOnFootIfOtherFootHasNoWidth());
   }
}
