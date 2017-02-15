package us.ihmc.exampleSimulations.skippy;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class SkippyV2Test
{

   private static final boolean sleepAfterTest = false;
   private SkippySimulationV2 skippySimulationV2;
   private SkippyRobotV2 skippy;

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      skippy.setQ_hip(0.1);
      skippy.setQ_shoulder(0.1);
      assertTrue(skippySimulationV2.run(10));
      System.out.println("testStanding");
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testRecoveringFromPush() throws SimulationExceededMaximumTimeException
   {
      double pushDuration = 0.03;
      Vector3d pushForce = new Vector3d(0.0, 0.0, -10.0);

      assertTrue(skippySimulationV2.run(1.0));
      pushRobot(pushDuration, pushForce);
      assertTrue(skippySimulationV2.run(5.0));
      System.out.println("testRecoveringFromPush");
   }

   private void pushRobot(double time, Vector3d force) throws SimulationExceededMaximumTimeException
   {
      SkippyRobotV2 skippy = skippySimulationV2.getSkippy();
      skippy.setRootJointForce(force.x, force.y, force.z);
      assertTrue(skippySimulationV2.run(time));
      skippy.setRootJointForce(0.0, 0.0, 0.0);
      System.out.println("pushRobot");
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
