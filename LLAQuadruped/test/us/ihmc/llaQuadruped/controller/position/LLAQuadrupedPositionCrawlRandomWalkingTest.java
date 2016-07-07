package us.ihmc.llaQuadruped.controller.position;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlRandomWalkingTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedPositionCrawlRandomWalkingTest extends QuadrupedPositionCrawlRandomWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 300.0)
   @Test(timeout = 1000000)
   public void testWalkingRandomly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingRandomly();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 200.0)
   @Test(timeout = 1000000)
   public void testWalkingAtRandomSpeedsWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingAtRandomSpeedsWithStops();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 300.0)
   @Test(timeout = 1000000)
   public void testWalkingRandomVelocitiesStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingRandomVelocitiesStoppingAndTurning();
   }
}
