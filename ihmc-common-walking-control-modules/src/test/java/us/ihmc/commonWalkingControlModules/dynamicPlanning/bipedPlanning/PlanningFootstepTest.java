package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.tools.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryGeneratorTestTools.getRandomPlanningFootstep;

public class PlanningFootstepTest
{
   @Test
   public void testSaveAndLoad()
   {
      Random random = new Random(1738L);
      PlanningFootstep stepB = new PlanningFootstep("", new YoRegistry("test"));

      for (int i = 0; i < 50; i++)
      {
         PlanningFootstep stepA = getRandomPlanningFootstep(random);

         stepB.loadValues(SaveableModuleStateTools.readSaveableRegistryToDataMap(SaveableModuleStateTools.writeStateToSaveableRegistry(stepA)));

         CoPTrajectoryGeneratorTestTools.assertFootstepEqual(stepA, stepB, 1e-7);
      }
   }
}
