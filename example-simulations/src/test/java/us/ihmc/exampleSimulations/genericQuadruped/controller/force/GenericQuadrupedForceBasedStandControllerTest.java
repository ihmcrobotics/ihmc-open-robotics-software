package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedSquaredUpInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceBasedStandControllerTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedForceBasedStandControllerTest extends QuadrupedForceBasedStandControllerTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      GenericQuadrupedTestFactory testFactory = new GenericQuadrupedTestFactory();
      testFactory.setInitialPosition(new GenericQuadrupedSquaredUpInitialPosition());

      return testFactory;
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 390000)
   public void testStandingUpAndAdjustingCoM() throws IOException
   {
      double comZShiftShift = 0.05;
      double comZDelta = 0.01;
      double orientationShift = Math.toRadians(5.0);
      double orientationDelta = Math.toRadians(1.0);

      super.testStandingUpAndAdjustingCoM(comZShiftShift, comZDelta, orientationShift, orientationDelta);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 550000)
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      super.testStandingAndResistingPushesOnBody();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 340000)
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontLeftHipRoll();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 320000)
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontRightHipRoll();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 340000)
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindLeftHipRoll();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 340000)
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindRightHipRoll();
   }
}
