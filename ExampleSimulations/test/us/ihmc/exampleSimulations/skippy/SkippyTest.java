package us.ihmc.exampleSimulations.skippy;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.exampleSimulations.skippy.SkippySimulation.SkippyControllerMode;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SkippyTest
{
   private static final SkippyControllerMode controllerMode = SkippyControllerMode.ICP_BASED;
   private static final boolean sleepAfterTest = false;

   private SkippySimulation skippySimulation;

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      SkippyRobot skippy = skippySimulation.getSkippy();
      skippy.getShoulderJoint().setQ(0.1);
      skippy.getHipJoint().setQ(0.1);

      assertTrue(skippySimulation.run(10.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testRecoveringFromPush() throws SimulationExceededMaximumTimeException
   {
      double pushDuration = 0.03;
      Vector3d pushForce = new Vector3d(0.0, 10.0, 0.0);

      assertTrue(skippySimulation.run(1.0));
      pushRobot(pushDuration, pushForce);
      assertTrue(skippySimulation.run(5.0));
   }

   private void pushRobot(double time, Vector3d force) throws SimulationExceededMaximumTimeException
   {
      SkippyRobot skippy = skippySimulation.getSkippy();
      skippy.setRootJointForce(force.x, force.y, force.z);
      assertTrue(skippySimulation.run(time));
      skippy.setRootJointForce(0.0, 0.0, 0.0);
   }

   @Before
   public void setupTest()
   {
      skippySimulation = new SkippySimulation(controllerMode);
   }

   @After
   public void afterTest()
   {
      if (sleepAfterTest)
         ThreadTools.sleepForever();
      skippySimulation.destroy();
   }
}
