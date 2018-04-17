package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitFlatGroundWalkingTest extends QuadrupedXGaitFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testWalkingForwardFast()
   {
      super.testFlatGroundWalking(90.0, 0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 720000)
   public void testWalkingForwardSlow()
   {
      super.testFlatGroundWalking(90.0, 0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testWalkingBackwardsFast()
   {
      super.testFlatGroundWalking(90.0, -0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testWalkingBackwardsSlow()
   {
      super.testFlatGroundWalking(90.0, -0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInASemiCircle(90.0, 0.6, 0.3);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInASemiCircle(90.0, 0.6, - 0.3);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInASemiCircle(90.0, -0.6, - 0.3);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInASemiCircle(90.0, -0.6, 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testTrottingForwardFast()
   {
      super.testFlatGroundWalking(180.0, 0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 720000)
   public void testTrottingForwardSlow()
   {
      super.testFlatGroundWalking(180.0, 0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testTrottingBackwardsFast()
   {
      super.testFlatGroundWalking(180.0, -0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testTrottingBackwardsSlow()
   {
      super.testFlatGroundWalking(180.0, -0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testTrottingInAForwardLeftCircle()
   {
      super.testWalkingInASemiCircle(180.0, 0.6, 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testTrottingInAForwardRightCircle()
   {
      super.testWalkingInASemiCircle(180.0, 0.6, - 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testTrottingInABackwardLeftCircle()
   {
      super.testWalkingInASemiCircle(180.0, -0.6, - 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testTrottingInABackwardRightCircle()
   {
      super.testWalkingInASemiCircle(180.0, -0.6, 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testPacingForwardFast()
   {
      super.testFlatGroundWalking(0.0, 0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 720000)
   public void testPacingForwardSlow()
   {
      super.testFlatGroundWalking(0.0, 0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testPacingBackwardsFast()
   {
      super.testFlatGroundWalking(0.0, -0.8);
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testPacingBackwardsSlow()
   {
      super.testFlatGroundWalking(0.0, -0.1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testPacingInAForwardLeftCircle()
   {
      super.testWalkingInASemiCircle(0.0, 0.6, 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 246.9)
   @Test(timeout = 1200000)
   public void testPacingInAForwardRightCircle()
   {
      super.testWalkingInASemiCircle(0.0, 0.6, - 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testPacingInABackwardLeftCircle()
   {
      super.testWalkingInASemiCircle(0.0, -0.6, - 0.3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testPacingInABackwardRightCircle()
   {
      super.testWalkingInASemiCircle(0.0, -0.6, 0.3);
   }
}
