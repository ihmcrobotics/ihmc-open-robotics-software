package us.ihmc.llaQuadruped.controller.force;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundTrotTest;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedXGaitFlatGroundTrotTest extends QuadrupedXGaitFlatGroundTrotTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 11.1)
   @Test(timeout = 55000)
   public void testTrottingForwardFast()
   {
      super.testTrottingForwardFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 13.5)
   @Test(timeout = 68000)
   public void testTrottingForwardSlow()
   {
      super.testTrottingForwardSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 12.7)
   @Test(timeout = 63000)
   public void testTrottingBackwardsFast()
   {
      super.testTrottingBackwardsFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 16.8)
   @Test(timeout = 84000)
   public void testTrottingBackwardsSlow()
   {
      super.testTrottingBackwardsSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 26.4)
   @Test(timeout = 130000)
   public void testTrottingInAForwardLeftCircle()
   {
      super.testTrottingInAForwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 24.1)
   @Test(timeout = 120000)
   public void testTrottingInAForwardRightCircle()
   {
      super.testTrottingInAForwardRightCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.8)
   @Test(timeout = 130000)
   public void testTrottingInABackwardLeftCircle()
   {
      super.testTrottingInABackwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 22.9)
   @Test(timeout = 110000)
   public void testTrottingInABackwardRightCircle()
   {
      super.testTrottingInABackwardRightCircle();
   }
}
