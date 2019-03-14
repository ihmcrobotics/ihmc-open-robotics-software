package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Test;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedPlanToWaypointTest;

public class GenericQuadrupedPlanToWaypointTest extends QuadrupedPlanToWaypointTest
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
