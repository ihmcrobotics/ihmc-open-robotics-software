package us.ihmc.llaQuadruped;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class LLAQuadrupedExampleTest
{
   private GoalOrientedTestConductor conductor;

   @Before
   public void setup()
   {
      try
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

         LLAQuadrupedTestFactory llaQuadrupedTestFactory = new LLAQuadrupedTestFactory();
         llaQuadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         llaQuadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);

         conductor = llaQuadrupedTestFactory.createTestConductor();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @DeployableTestMethod(estimatedDuration = 10.0)
   @Test(timeout = 800000)
   public void exampleTest()
   {
      conductor.simulate();
   }
}
