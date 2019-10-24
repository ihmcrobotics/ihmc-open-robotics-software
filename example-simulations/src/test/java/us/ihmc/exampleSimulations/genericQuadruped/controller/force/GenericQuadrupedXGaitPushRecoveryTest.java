package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Tag;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitPushRecoveryTest;

@Tag("quadruped-xgait")
public class GenericQuadrupedXGaitPushRecoveryTest extends QuadrupedXGaitPushRecoveryTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   public double getWalkingSpeed()
   {
      return 0.8;
   }

   @Override
   public double getStepDuration()
   {
      return 0.35;
   }
}
