package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlWalkingForwardStoppingAndTurningTest;
import us.ihmc.simulationConstructionSet.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class GenericQuadrupedPositionCrawlWalkingForwardStoppingAndTurningTest extends QuadrupedPositionCrawlWalkingForwardStoppingAndTurningTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 700.0)
   @Test(timeout = 2000000)
   public void testWalkingForwardStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardStoppingAndTurning();
   }
}
