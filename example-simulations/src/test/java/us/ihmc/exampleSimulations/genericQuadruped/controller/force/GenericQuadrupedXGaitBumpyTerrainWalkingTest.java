package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitBumpyTerrainWalkingTest;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitBumpyTerrainWalkingTest extends QuadrupedXGaitBumpyTerrainWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 250000)
   public void testWalkingOverShallowBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverShallowBumpyTerrain();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 40.0)
   @Test(timeout = 250000)
   public void testWalkingOverMediumBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverMediumBumpyTerrain();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 350000)
   public void testWalkingOverAggressiveBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverAggressiveBumpyTerrain();
   }
}
