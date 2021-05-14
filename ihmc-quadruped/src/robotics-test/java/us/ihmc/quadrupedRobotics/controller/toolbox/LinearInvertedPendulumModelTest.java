package us.ihmc.quadrupedRobotics.controller.toolbox;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;

public class LinearInvertedPendulumModelTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testLinearInvertedPendulumModelGettersAndSetters()
   {
      TranslationReferenceFrame comZUpFrame = new TranslationReferenceFrame("comZUpFrame", ReferenceFrame.getWorldFrame());
      YoRegistry registry = new YoRegistry("testLinearInvertedPendulumModelGettersAndSetters");
      
      double mass = 100.0;
      double gravity = 9.81;
      double comHeight = 2.0;
      
      LinearInvertedPendulumModel lipModel = new LinearInvertedPendulumModel(comZUpFrame, mass, gravity, comHeight, registry);
      
      double epsilon = 1e-7;
      assertEquals(lipModel.getLipmHeight(), comHeight, epsilon);
      assertEquals(lipModel.getGravity(), gravity, epsilon);
      assertEquals(lipModel.getMass(), mass, epsilon);
      
      comHeight--;
      lipModel.setLipmHeight(comHeight);
      assertEquals(lipModel.getLipmHeight(), comHeight, epsilon);
      
//      gravity++;
//      lipModel.setGravity(gravity);
//      assertEquals(lipModel.getGravity(), gravity, epsilon);
      
      mass++;
      lipModel.setMass(mass);
      assertEquals(lipModel.getMass(), mass, epsilon);
      
      //assumes comHeight is 1
      assertEquals(lipModel.getNaturalFrequency(), Math.sqrt(gravity), epsilon);
      
      comHeight = 50;
      lipModel.setLipmHeight(comHeight);
      assertEquals(lipModel.getLipmHeight(), comHeight, epsilon);
      assertEquals(lipModel.getNaturalFrequency(), Math.sqrt(gravity / comHeight), epsilon);
      assertEquals(lipModel.getTimeConstant(), 1.0 / Math.sqrt(gravity / comHeight), epsilon);
   }
}
