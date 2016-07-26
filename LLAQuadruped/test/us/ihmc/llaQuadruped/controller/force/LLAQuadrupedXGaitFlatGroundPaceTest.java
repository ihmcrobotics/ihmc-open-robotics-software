package us.ihmc.llaQuadruped.controller.force;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundPaceTest;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedXGaitFlatGroundPaceTest extends QuadrupedXGaitFlatGroundPaceTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 11.1)
   @Test(timeout = 55000)
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 13.5)
   @Test(timeout = 68000)
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 12.7)
   @Test(timeout = 63000)
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 16.8)
   @Test(timeout = 84000)
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 26.4)
   @Test(timeout = 130000)
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 24.1)
   @Test(timeout = 120000)
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.8)
   @Test(timeout = 130000)
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 22.9)
   @Test(timeout = 110000)
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }
}
