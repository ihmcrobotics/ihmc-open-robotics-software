package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class FootstepPostProcessingKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey splitFractionProcessingEnabled = keys.addBooleanKey("Split fraction processing enabled", true);

}
