package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitBumpyTerrainWalkingTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@Disabled // TODO Need to either change the ground profile to using a TerrainObject3D or add support for height map in SCS2
public class GenericQuadrupedXGaitBumpyTerrainWalkingTest extends QuadrupedXGaitBumpyTerrainWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   @Tag("quadruped-xgait-slow")
   @Test
   public void testWalkingOverShallowBumpyTerrain() throws IOException
   {
      super.testWalkingOverShallowBumpyTerrain();
   }

   @Override
   @Tag("quadruped-xgait")
   @Test
   public void testWalkingOverMediumBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingOverMediumBumpyTerrain();
   }

   @Override
   @Tag("quadruped-xgait")
   @Test
   public void testTrottingOverAggressiveBumpyTerrain() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTrottingOverAggressiveBumpyTerrain();
   }
}
