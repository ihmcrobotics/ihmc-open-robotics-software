package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Test;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedBodyPathPlanTest;

public class GenericQuadrupedBodyPathPlanTest extends QuadrupedBodyPathPlanTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Test
   @Override
   public void testSimpleBodyPathPlan()
   {
      super.testSimpleBodyPathPlan();
   }

   @Test
   @Override
   public void testBodyPathAroundASimpleMaze()
   {
      super.testBodyPathAroundASimpleMaze();
   }
}
