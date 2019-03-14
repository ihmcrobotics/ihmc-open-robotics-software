package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Test;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitBumpyTerrainWalkingTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

public class GenericQuadrupedXGaitBumpyTerrainWalkingTest extends QuadrupedXGaitBumpyTerrainWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   @Test
   public void testWalkingOverShallowBumpyTerrain() throws IOException
   {
      super.testWalkingOverShallowBumpyTerrain();
   }

   @Override
   @Test
   public void testWalkingOverMediumBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverMediumBumpyTerrain();
   }

   @Override
   @Test
   public void testTrottingOverAggressiveBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTrottingOverAggressiveBumpyTerrain();
   }
}
