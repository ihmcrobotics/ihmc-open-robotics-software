package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;

public class GenericQuadrupedXGaitFlatGroundWalkingTest extends QuadrupedXGaitFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }

   // Flaky
   @Disabled
   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }

   // Flaky
   @Disabled
   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingForwardFast()
   {
      super.testTrottingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingForwardSlow()
   {
      super.testTrottingForwardSlow();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingBackwardsFast()
   {
      super.testTrottingBackwardsFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingBackwardsSlow()
   {
      super.testTrottingBackwardsSlow();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingInAForwardLeftCircle()
   {
      super.testTrottingInAForwardLeftCircle();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingInAForwardRightCircle()
   {
      super.testTrottingInAForwardRightCircle();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingInABackwardLeftCircle()
   {
      super.testTrottingInABackwardLeftCircle();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingInABackwardRightCircle()
   {
      super.testTrottingInABackwardRightCircle();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }

   @Override
   public double getPacingWidth()
   {
      return 0.2;
   }

   @Override
   public double getPacingStepDuration()
   {
      return 0.3;
   }

   @Override
   public double getPacingEndDoubleSupportDuration()
   {
      return 0.05;
   }

   @Override
   public double getFastWalkingSpeed()
   {
      return 0.8;
   }

   @Override
   public double getSlowWalkingSpeed()
   {
      return 0.1;
   }

   @Override
   public double getWalkingAngularVelocity()
   {
      return 0.3;
   }

   @Override
   public double getWalkingSpeedWhileTurning()
   {
      return 0.4;
   }
}
