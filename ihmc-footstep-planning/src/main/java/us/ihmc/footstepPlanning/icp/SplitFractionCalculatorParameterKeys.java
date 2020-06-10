package us.ihmc.footstepPlanning.icp;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class SplitFractionCalculatorParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey  defaultTransferSplitFraction            = keys.addDoubleKey("Default transfer split fraction", 0.5);
   public static final DoubleStoredPropertyKey  stepHeightForLargeStepDown              = keys.addDoubleKey("Step height for large step down", 0.15);
   public static final DoubleStoredPropertyKey  largestStepDownHeight                   = keys.addDoubleKey("Largest step down height", 0.25);
   public static final DoubleStoredPropertyKey  transferSplitFractionAtFullDepth        = keys.addDoubleKey("Transfer split fraction at full depth", 0.3);
   public static final DoubleStoredPropertyKey  transferWeightDistributionAtFullDepth   = keys.addDoubleKey("Transfer weight distribution at full depth", 0.75);

   public static final DoubleStoredPropertyKey  fractionLoadIfFootHasFullSupport        = keys.addDoubleKey("Fraction load if foot has full support", 0.5);
   public static final DoubleStoredPropertyKey  fractionTimeOnFootIfFootHasFullSupport  = keys.addDoubleKey("Fraction time on foot if foot has full support", 0.5);
   public static final DoubleStoredPropertyKey  fractionLoadIfOtherFootHasNoWidth       = keys.addDoubleKey("Fraction load if other foot has no width", 0.5);
   public static final DoubleStoredPropertyKey  fractionTimeOnFootIfOtherFootHasNoWidth = keys.addDoubleKey("Fraction time on foot if other foot has no width", 0.5);
}
