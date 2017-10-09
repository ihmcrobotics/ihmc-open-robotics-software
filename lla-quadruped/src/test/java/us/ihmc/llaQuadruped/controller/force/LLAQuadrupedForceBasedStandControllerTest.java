package us.ihmc.llaQuadruped.controller.force;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceBasedStandControllerTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class LLAQuadrupedForceBasedStandControllerTest extends QuadrupedForceBasedStandControllerTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingUpAndAdjustingCoM() throws IOException
   {
      super.testStandingUpAndAdjustingCoM();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      super.testStandingAndResistingPushesOnBody();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontLeftHipRoll();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontRightHipRoll();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindLeftHipRoll();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindRightHipRoll();
   }
}
