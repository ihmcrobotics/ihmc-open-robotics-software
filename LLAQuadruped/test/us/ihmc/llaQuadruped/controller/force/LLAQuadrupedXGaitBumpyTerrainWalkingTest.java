package us.ihmc.llaQuadruped.controller.force;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitBumpyTerrainWalkingTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedXGaitBumpyTerrainWalkingTest extends QuadrupedXGaitBumpyTerrainWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testWalkingOverShallowBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverShallowBumpyTerrain();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 120000)
   public void testWalkingOverMediumBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverMediumBumpyTerrain();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 120000)
   public void testWalkingOverAggressiveBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverAggressiveBumpyTerrain();
   }
}
