package us.ihmc.llaQuadruped.controller.force;

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
   @DeployableTestMethod(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingUpAndAdjustingCoM()
   {
      super.testStandingUpAndAdjustingCoM();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingAndResistingPushes()
   {
      super.testStandingAndResistingPushes();
   }
}
