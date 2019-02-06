package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitPushRecoveryTest;

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
