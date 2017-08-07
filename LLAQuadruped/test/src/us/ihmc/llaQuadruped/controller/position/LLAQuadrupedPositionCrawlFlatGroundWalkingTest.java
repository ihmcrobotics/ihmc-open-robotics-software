package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlFlatGroundWalkingTest;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedPositionCrawlFlatGroundWalkingTest extends QuadrupedPositionCrawlFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingForwardFast() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingForwardSlow() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingBackwardsFast() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingBackwardsSlow() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInAForwardLeftCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInAForwardRightCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInABackwardLeftCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInABackwardRightCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInABackwardRightCircle();
   }
}
