package us.ihmc.exampleSimulations.skippy;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SkippyTest
{
   private static final boolean sleepAfterTest = false;
   private SkippySimulation skippySimulation;

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      assertTrue(skippySimulation.run(10.0));
   }

   @Before
   public void setupTest()
   {
      skippySimulation = new SkippySimulation();
   }

   @After
   public void afterTest()
   {
      if (sleepAfterTest)
         ThreadTools.sleepForever();
      skippySimulation.destroy();
   }
}
