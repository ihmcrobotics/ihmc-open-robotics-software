package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryGeneratorTestTools.getRandomPlanningFootstep;

public class DynamicPlanningFootstepTest
{
   @Test
   public void testSaveAndLoad()
   {
      Random random = new Random(1738L);
      DynamicPlanningFootstep stepB = new DynamicPlanningFootstep("", new YoRegistry("test"));

      for (int i = 0; i < 50; i++)
      {
         DynamicPlanningFootstep stepA = getRandomPlanningFootstep(random);

         stepB.loadValues(YoSaveableModuleStateTools.readSaveableRegistryToDataMap(YoSaveableModuleStateTools.writeStateToSaveableRegistry(stepA)));

         CoPTrajectoryGeneratorTestTools.assertFootstepEqual(stepA, stepB, 1e-7);
      }
   }
}
