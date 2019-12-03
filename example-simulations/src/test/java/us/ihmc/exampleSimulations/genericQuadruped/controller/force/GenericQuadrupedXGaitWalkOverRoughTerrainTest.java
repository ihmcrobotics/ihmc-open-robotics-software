package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import java.io.IOException;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkOverRoughTerrainTest;

public class GenericQuadrupedXGaitWalkOverRoughTerrainTest extends QuadrupedXGaitWalkOverRoughTerrainTest
{
   private QuadrupedXGaitSettingsReadOnly xGaitSettings;

   @Tag("quadruped-xgait")
   @Override
   @Test
   public void testWalkingUpStaircase() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingUpStaircase();
   }

   @Tag("quadruped-xgait")
   @Override
   @Test
   public void testWalkingOverTiledGround() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverTiledGround();
   }

   @Tag("quadruped-xgait")
   @Test
   public void testWalkingOverSingleStepUp() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverSingleStepUp(Double.NaN);
   }

   @Tag("quadruped-xgait")
   @Override
   @Test
   public void testWalkingOverConsecutiveRamps() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverConsecutiveRamps();
   }

   @Override
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
