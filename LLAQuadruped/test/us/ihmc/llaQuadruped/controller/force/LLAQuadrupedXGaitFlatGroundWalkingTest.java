package us.ihmc.llaQuadruped.controller.force;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedXGaitFlatGroundWalkingTest extends QuadrupedXGaitFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }
}
