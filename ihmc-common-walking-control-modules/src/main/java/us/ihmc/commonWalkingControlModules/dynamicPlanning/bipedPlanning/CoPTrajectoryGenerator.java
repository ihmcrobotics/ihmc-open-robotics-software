package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class CoPTrajectoryGenerator extends YoSaveableModule<CoPTrajectoryGeneratorState>
{
   public CoPTrajectoryGenerator(Class<? extends YoSaveableModule> moduleName, YoRegistry registry)
   {
      super(moduleName.getSimpleName(), registry);
   }

   public CoPTrajectoryGenerator(String moduleName, YoRegistry registry)
   {
      super(moduleName, registry);
   }

   public abstract RecyclingArrayList<SettableContactStateProvider> getContactStateProviders();
}
