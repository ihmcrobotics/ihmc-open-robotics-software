package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkOverRoughTerrainTest;

import java.io.IOException;

public class GenericQuadrupedXGaitWalkOverRoughTerrainTest extends QuadrupedXGaitWalkOverRoughTerrainTest
{
   private QuadrupedXGaitSettingsReadOnly xGaitSettings;

   @Test
   public void testWalkingUpStaircase() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingUpStaircase();
   }

   @Test
   public void testWalkingOverTiledGround() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverTiledGround();
   }

   @Test
   public void testWalkingOverSingleStepUp() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverSingleStepUp(Double.NaN);
   }

   @Test
   public void testWalkingOverConsecutiveRamps() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverConsecutiveRamps();
   }

   @Disabled
   @Test
   public void testWalkingOverCinderBlockField() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverCinderBlockField();
   }

   @Override
   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      return xGaitSettings;
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
}
