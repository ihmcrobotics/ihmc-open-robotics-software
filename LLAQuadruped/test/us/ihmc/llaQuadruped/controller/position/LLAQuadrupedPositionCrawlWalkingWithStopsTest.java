package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlWalkingWithStopsTest;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedPositionCrawlWalkingWithStopsTest extends QuadrupedPositionCrawlWalkingWithStopsTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
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
