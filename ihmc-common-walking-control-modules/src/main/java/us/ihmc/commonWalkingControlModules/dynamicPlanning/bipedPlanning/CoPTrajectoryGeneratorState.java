package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.saveableModule.SaveableModuleState;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoPTrajectoryGeneratorState extends SaveableModuleState
{
   private final RecyclingArrayList<PlanningFootstep> footsteps;

   public CoPTrajectoryGeneratorState(YoRegistry registry)
   {
      footsteps = new RecyclingArrayList<>(3, () -> createFootstep(registry));
   }

   private PlanningFootstep createFootstep(YoRegistry registry)
   {
      return new PlanningFootstep("" + footsteps.size(), registry);
   }
}
