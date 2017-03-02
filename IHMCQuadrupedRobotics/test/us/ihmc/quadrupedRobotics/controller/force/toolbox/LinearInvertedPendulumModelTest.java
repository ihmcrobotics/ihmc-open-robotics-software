package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;

public class LinearInvertedPendulumModelTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInvertedPendulumModelGettersAndSetters()
   {
      TranslationReferenceFrame comZUpFrame = new TranslationReferenceFrame("comZUpFrame", ReferenceFrame.getWorldFrame());
      YoVariableRegistry registry = new YoVariableRegistry("testLinearInvertedPendulumModelGettersAndSetters");
      
      double mass = 100.0;
      double gravity = 9.81;
      double comHeight = 2.0;
      
      LinearInvertedPendulumModel lipModel = new LinearInvertedPendulumModel(comZUpFrame, mass, gravity, comHeight, registry);
      
      double epsilon = 1e-7;
      assertEquals(lipModel.getComHeight(), comHeight, epsilon);
      assertEquals(lipModel.getGravity(), gravity, epsilon);
      assertEquals(lipModel.getMass(), mass, epsilon);
      
      comHeight--;
      lipModel.setComHeight(comHeight);
      assertEquals(lipModel.getComHeight(), comHeight, epsilon);
      
      gravity++;
      lipModel.setGravity(gravity);
      assertEquals(lipModel.getGravity(), gravity, epsilon);
      
      mass++;
      lipModel.setMass(mass);
      assertEquals(lipModel.getMass(), mass, epsilon);
      
      //assumes comHeight is 1
      assertEquals(lipModel.getNaturalFrequency(), Math.sqrt(gravity), epsilon);
      
      comHeight = 50;
      lipModel.setComHeight(comHeight);
      assertEquals(lipModel.getComHeight(), comHeight, epsilon);
      assertEquals(lipModel.getNaturalFrequency(), Math.sqrt(gravity / comHeight), epsilon);
      assertEquals(lipModel.getTimeConstant(), 1.0 / Math.sqrt(gravity / comHeight), epsilon);
   }
}
