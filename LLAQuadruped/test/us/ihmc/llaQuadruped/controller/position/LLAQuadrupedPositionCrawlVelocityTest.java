package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlVelocityTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;

@ContinuousIntegrationPlan(targets = TestPlanTarget.InDevelopment)
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
