package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlTurning360Test;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedPositionCrawlTurning360Test extends QuadrupedPositionCrawlTurning360Test
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 120.0)
   @Test(timeout = 800000)
   public void rotate360InPlaceRight() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.rotate360InPlaceRight();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 120.0)
   @Test(timeout = 800000)
   public void rotate360InPlaceLeft() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.rotate360InPlaceLeft();
   }
}
