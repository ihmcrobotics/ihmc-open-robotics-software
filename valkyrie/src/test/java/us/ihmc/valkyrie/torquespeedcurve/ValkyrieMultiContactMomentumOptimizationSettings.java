package us.ihmc.valkyrie.torquespeedcurve;

import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieMomentumOptimizationSettings;

public class ValkyrieMultiContactMomentumOptimizationSettings extends ValkyrieMomentumOptimizationSettings
{
   public ValkyrieMultiContactMomentumOptimizationSettings(ValkyrieJointMap jointMap)
   {
      super(jointMap);
   }

   @Override
   public int getNumberOfContactableBodies()
   {
      return 6;
   }
}
