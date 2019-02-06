package us.ihmc.exampleSimulations.skippy;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.skippy.SkippySimulation.SkippyControllerMode;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.commons.thread.ThreadTools;

public class SkippyTest
{
   private static final SkippyControllerMode controllerMode = SkippyControllerMode.ICP_BASED;
   private static final boolean sleepAfterTest = false;

   private SkippySimulation skippySimulation;

   
   @Disabled
   @Test
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      SkippyRobot skippy = skippySimulation.getSkippy();
      skippy.getShoulderJoint().setQ(0.1);
      skippy.getHipJoint().setQ(0.1);

      assertTrue(skippySimulation.run(10.0));
   }

   @Test
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

   @BeforeEach
   public void setupTest()
   {
      skippySimulation = new SkippySimulation(controllerMode);
   }

   @AfterEach
   public void afterTest()
   {
      if (sleepAfterTest)
         ThreadTools.sleepForever();
      skippySimulation.destroy();
   }
}
