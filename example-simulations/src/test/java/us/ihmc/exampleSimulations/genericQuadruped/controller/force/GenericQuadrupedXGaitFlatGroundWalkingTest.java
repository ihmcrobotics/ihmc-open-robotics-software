package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

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
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingBackwardsFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingForwardFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingForwardSlow()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingBackwardsFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingBackwardsSlow()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingInAForwardLeftCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingInAForwardRightCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testTrottingInABackwardLeftCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testTrottingInABackwardRightCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingForwardFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingForwardSlow()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingBackwardsFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingBackwardsSlow()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingInAForwardLeftCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingInAForwardRightCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-2")
   @Override
   @Test
   public void testPacingInABackwardLeftCircle()
   {
      super.testWalkingForwardFast();
   }

   @Tag("quadruped-xgait-slow-2")
   @Override
   @Test
   public void testPacingInABackwardRightCircle()
   {
      super.testWalkingForwardFast();
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
