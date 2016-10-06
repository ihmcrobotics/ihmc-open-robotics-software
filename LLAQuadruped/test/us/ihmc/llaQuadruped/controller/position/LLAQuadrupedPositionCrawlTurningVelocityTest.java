package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlTurningVelocityTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;

@ContinuousIntegrationPlan(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedPositionCrawlTurningVelocityTest extends QuadrupedPositionCrawlTurningVelocityTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testTurnInPlaceRegularSpeed() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTurnInPlaceRegularSpeed();
   }
   
   //"Turn in place slowly still fails due to CoM shifting outside support polygon. Need to fix it..."
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testTurnInPlaceSlowly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTurnInPlaceSlowly();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 500.0)
   @Test(timeout = 2000000)
   public void testWalkingBackwardStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardStoppingAndTurning();
   }
}
