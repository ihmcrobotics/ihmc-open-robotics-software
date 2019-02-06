package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import org.junit.jupiter.api.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlRandomWalkingTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@Disabled
public class GenericQuadrupedPositionCrawlRandomWalkingTest extends QuadrupedPositionCrawlRandomWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @Test
   public void testWalkingRandomly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingRandomly();
   }

   @Override
   @Test
   public void testWalkingAtRandomSpeedsWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingAtRandomSpeedsWithStops();
   }

   @Override
   @Test
   public void testWalkingRandomVelocitiesStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingRandomVelocitiesStoppingAndTurning();
   }
}
