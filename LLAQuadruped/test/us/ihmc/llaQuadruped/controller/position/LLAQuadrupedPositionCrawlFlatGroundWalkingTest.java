package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlFlatGroundWalkingTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedPositionCrawlFlatGroundWalkingTest extends QuadrupedPositionCrawlFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingForwardFast() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingForwardSlow() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingBackwardsFast() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 33.0)
   @Test(timeout = 200000)
   public void testWalkingBackwardsSlow() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInAForwardLeftCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInAForwardRightCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInABackwardLeftCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 70.0)
   @Test(timeout = 200000)
   public void testWalkingInABackwardRightCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInABackwardRightCircle();
   }
}
