package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.tools.saveableModule.SaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class CoPTrajectoryGenerator extends SaveableModule<CoPTrajectoryGeneratorState>
{
   public CoPTrajectoryGenerator(Class<? extends SaveableModule> moduleName, YoRegistry registry)
   {
      super(moduleName.getSimpleName(), registry);
   }

   public CoPTrajectoryGenerator(String moduleName, YoRegistry registry)
   {
      super(moduleName, registry);
   }

   public abstract RecyclingArrayList<SettableContactStateProvider> getContactStateProviders();
}
