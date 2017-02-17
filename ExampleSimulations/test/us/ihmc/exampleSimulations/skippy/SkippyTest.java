package us.ihmc.exampleSimulations.skippy;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.skippy.SkippySimulation.SkippyControllerMode;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SkippyTest
{
   private static final SkippyControllerMode controllerMode = SkippyControllerMode.ICP_BASED;
   private static final boolean sleepAfterTest = false;

   private SkippySimulation skippySimulation;

   
   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
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
      Vector3D pushForce = new Vector3D(0.0, 10.0, 0.0);

      assertTrue(skippySimulation.run(1.0));
      pushRobot(pushDuration, pushForce);
      assertTrue(skippySimulation.run(5.0));
   }

   private void pushRobot(double time, Vector3D force) throws SimulationExceededMaximumTimeException
   {
      SkippyRobot skippy = skippySimulation.getSkippy();
      skippy.setRootJointForce(force.getX(), force.getY(), force.getZ());
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
