package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Test;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.planning.AStarPawStepSimulationPlanToWaypointTest;

public class GenericQuadrupedAStarPawStepPlannerTest extends AStarPawStepSimulationPlanToWaypointTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Test
   @Override
   public void testSimpleForwardPoint()
   {
      super.testSimpleForwardPoint();
   }

}
