package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class FootstepPostProcessingKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey splitFractionProcessingEnabled = keys.addBooleanKey("Split fraction processing enabled", true);

   public static final DoubleStoredPropertyKey stepHeightForLargeStepDown = keys.addDoubleKey("Step height for large step down", 0.15);
   public static final DoubleStoredPropertyKey largestStepDownHeight = keys.addDoubleKey("Largest step down height", 0.35);
   public static final DoubleStoredPropertyKey transferSplitFractionAtFullDepth = keys.addDoubleKey("Transfer split fraction at full depth", 0.2);
   public static final DoubleStoredPropertyKey transferWeightDistributionAtFullDepth = keys.addDoubleKey("Transfer weight distribution at full depth", 0.7);
}
