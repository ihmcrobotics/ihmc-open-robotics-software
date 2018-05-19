package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlWalkingWithStopsTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class GenericQuadrupedPositionCrawlWalkingWithStopsTest extends QuadrupedPositionCrawlWalkingWithStopsTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingForwardFastWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardFastWithStops();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 120.0)
   @Test(timeout = 600000)
   public void testWalkingForwardSlowWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardSlowWithStops();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingBackwardSlowWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardSlowWithStops();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingBackwardFastWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardFastWithStops();
   }
}
