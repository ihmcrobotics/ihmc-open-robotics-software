package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

@Disabled
public class BipedCoMTrajectoryPlannerTest
{
   @Disabled
   // TODO GITHUB WORKFLOWS
   // I couldn't get this test to start automatically
   @Test
   public void testFancySteps()
   {
      new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createFancySteps);
   }

   @Disabled
   // TODO GITHUB WORKFLOWS
   // I couldn't get this test to start automatically
   @Test
   public void testRunningSteps()
   {
      new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createSteps);
   }
}
