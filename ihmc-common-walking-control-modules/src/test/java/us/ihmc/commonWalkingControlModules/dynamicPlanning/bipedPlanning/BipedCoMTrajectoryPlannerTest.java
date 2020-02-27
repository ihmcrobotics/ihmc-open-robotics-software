package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;

public class BipedCoMTrajectoryPlannerTest
{
   @Test
   public void testFancySteps()
   {
      new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createFancySteps);
   }

   @Test
   public void testRunningSteps()
   {
      new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createSteps);
   }
}
