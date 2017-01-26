package us.ihmc.exampleSimulations.skippy;

import org.junit.Test;

import us.ihmc.exampleSimulations.skippy.SkippySimulation.SkippyControllerMode;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SkippyV2Test
{

   private static final SkippyControllerMode controllerMode = SkippyControllerMode.ICP_BASED;
   final boolean sleepAfterTest = false;

   SkippySimulationV2 skippySimulationV2;

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      SkippyRobotV2 skippyV2 = skippySimulationV2.getSkippy();
      //		      skippyV2.getShoulderJoint().setQ(0.1);
      //		      skippyV2.getHipJoint().setQ(0.1);
      //
      //		      assertTrue(skippySimulation.run(10.0));
   }
}
