package us.ihmc.exampleSimulations.skippy;

import static org.junit.Assert.*;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.exampleSimulations.skippy.SkippySimulationV2;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SkippyV2Test
{

   //   private static final SkippyControllerMode controllerMode = SkippyControllerMode.ICP_BASED;
   final boolean sleepAfterTest = true;

   double flexHipDuration = 0.03;
   double flexHipAngle = Math.toRadians(2.0);

   SkippySimulationV2 skippySimulationV2;
   SkippyRobotV2 skippy;

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   //@Test(timeout = 100000)
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      skippy.setQ_hip(flexHipAngle);
      assertTrue(skippySimulationV2.run(flexHipDuration));
      skippy.setQ_hip(0.0);
      assertTrue(skippySimulationV2.run(10.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 100000)
   public void testRecoveringFromPush() throws SimulationExceededMaximumTimeException
   {
      double pushDuration = 0.03;
      Vector3d pushForce = new Vector3d(0.0, 10.0, 0.0);

      assertTrue(skippySimulationV2.run(1.0));
      pushRobot(pushDuration, pushForce);
      assertTrue(skippySimulationV2.run(5.0));
   }

   private void pushRobot(double time, Vector3d force) throws SimulationExceededMaximumTimeException
   {
      skippy.setRootJointForce(force.x, force.y, force.z);
      assertTrue(skippySimulationV2.run(time));
      skippy.setRootJointForce(0.0, 0.0, 0.0);
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
