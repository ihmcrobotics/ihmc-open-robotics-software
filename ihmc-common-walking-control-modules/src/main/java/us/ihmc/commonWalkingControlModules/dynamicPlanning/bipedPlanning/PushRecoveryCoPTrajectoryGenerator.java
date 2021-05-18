package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class PushRecoveryCoPTrajectoryGenerator extends YoSaveableModule<PushRecoveryState>
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   public PushRecoveryCoPTrajectoryGenerator(YoRegistry parentRegistry)
   {
      super(PushRecoveryCoPTrajectoryGenerator.class, parentRegistry);
   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   public List<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   @Override
   public void compute(PushRecoveryState state)
   {
      contactStateProviders.clear();
   }
}
