package us.ihmc.llaQuadruped.controller.force;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceBasedStandControllerTest;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedForceBasedStandControllerTest extends QuadrupedForceBasedStandControllerTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingUpAndAdjustingCoM() throws IOException
   {
      super.testStandingUpAndAdjustingCoM();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      super.testStandingAndResistingPushesOnBody();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontLeftHipRoll();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontRightHipRoll();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindLeftHipRoll();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindRightHipRoll();
   }
}
