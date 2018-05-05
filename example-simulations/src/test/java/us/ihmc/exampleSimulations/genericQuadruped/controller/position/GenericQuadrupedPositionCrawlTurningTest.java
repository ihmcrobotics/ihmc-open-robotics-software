package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlTurningTest;
import us.ihmc.simulationConstructionSet.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class GenericQuadrupedPositionCrawlTurningTest extends QuadrupedPositionCrawlTurningTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 42.0)
   @Test(timeout = 800000)
   public void testYawingRightFastNinetyDegrees() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testYawingRightFastNinetyDegrees();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 42.0)
   @Test(timeout = 800000)
   public void testYawingLeftFastNinetyDegrees() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testYawingLeftFastNinetyDegrees();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 800000)
   public void testYawingRightSlowNinetyDegrees() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testYawingRightSlowNinetyDegrees();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 800000)
   public void testYawingLeftSlowNinetyDegrees() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testYawingLeftSlowNinetyDegrees();
   }
}
