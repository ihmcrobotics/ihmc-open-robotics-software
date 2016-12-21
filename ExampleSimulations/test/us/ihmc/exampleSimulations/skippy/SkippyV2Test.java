package us.ihmc.exampleSimulations.skippy;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.exampleSimulations.skippy.SkippySimulationV2;//.SkippyControllerMode;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SkippyV2Test
{

   //   private static final SkippyControllerMode controllerMode = SkippyControllerMode.ICP_BASED;
   final boolean sleepAfterTest = false;

   SkippySimulationV2 skippySimulationV2;// = new SkippySimulationV2();
   SkippyRobotV2 skippy;// = skippySimulationV2.getSkippy();

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      //      skippy.getShoulderJoint().setQ(0.1);
      //      skippy.getHipJoint().setQ(0.1);
      skippy.setQ_hip(0.1);

      assertTrue(skippySimulationV2.run(10.0));
   }

   @Before
   public void setupTest()
   {
      skippySimulationV2 = new SkippySimulationV2();
      skippy = skippySimulationV2.getSkippy();
   }

   @After
   public void afterTest()
   {
      if (sleepAfterTest)
         ThreadTools.sleepForever();
      skippySimulationV2.destroy();
   }

}
