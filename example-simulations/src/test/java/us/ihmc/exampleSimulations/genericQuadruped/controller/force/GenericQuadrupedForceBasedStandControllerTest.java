package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedSquaredUpInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceBasedStandControllerTest;

public class GenericQuadrupedForceBasedStandControllerTest extends QuadrupedForceBasedStandControllerTest
{
   public double getHeightShift()
   {
      return 0.05;
   }

   public double getHeightDelta()
   {
      return 0.01;
   }

   public double getTranslationShift()
   {
      return 0.07;
   }

   public double getTranslationDelta()
   {
      return 0.01;
   }

   public double getOrientationShift()
   {
      return Math.toRadians(5.0);
   }

   public double getOrientationDelta()
   {
      return Math.toRadians(1.0);
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      GenericQuadrupedTestFactory testFactory = new GenericQuadrupedTestFactory();
      testFactory.setInitialPosition(new GenericQuadrupedSquaredUpInitialPosition());

      return testFactory;
   }


}
