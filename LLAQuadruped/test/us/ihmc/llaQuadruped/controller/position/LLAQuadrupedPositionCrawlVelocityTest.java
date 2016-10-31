package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlVelocityTest;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedPositionCrawlVelocityTest extends QuadrupedPositionCrawlVelocityTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 600000)
   public void testWalkingForward() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForward();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 600000)
   public void testWalkingBackward() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackward();
   }
}
