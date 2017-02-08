package us.ihmc.robotics.alphaToAlpha;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * Created by Peter on 9/11/2016.
 */
public class YoMiniJerkUpAndDownAlphaToAlphaTest
{
   private double EPSILON = 1e-6;
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testInvalidYoVariables()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      DoubleYoVariable startOfRampUp = new DoubleYoVariable("startOfRampUp", registry);
      DoubleYoVariable endOfRamp = new DoubleYoVariable("endOfRamp", registry);
      DoubleYoVariable startOfRampDown = new DoubleYoVariable("startOfRampDown", registry);
      DoubleYoVariable endOfRampDown = new DoubleYoVariable("endOfRampDown", registry);

      YoMiniJerkUpAndDownAlphaToAlpha yoMiniJerkUpAndDownAlphaToAlpha = new YoMiniJerkUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      testRangeOfAlphas(0.0, yoMiniJerkUpAndDownAlphaToAlpha);

      startOfRampUp.set(0.1);
      testRangeOfAlphas(0.0, yoMiniJerkUpAndDownAlphaToAlpha);

      endOfRamp.set(0.2);
      testRangeOfAlphas(0.0, yoMiniJerkUpAndDownAlphaToAlpha);

      startOfRampDown.set(0.3);
      testRangeOfAlphas(0.0, yoMiniJerkUpAndDownAlphaToAlpha);

      endOfRampDown.set(1.0);
      testRangeOfAlphas(0.0, yoMiniJerkUpAndDownAlphaToAlpha);

      endOfRampDown.set(0.9);
      startOfRampDown.set(0.95);
      testRangeOfAlphas(0.0, yoMiniJerkUpAndDownAlphaToAlpha);
   }



   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testValidYoVariables()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      DoubleYoVariable startOfRampUp = new DoubleYoVariable("startOfRampUp", registry);
      DoubleYoVariable endOfRamp = new DoubleYoVariable("endOfRamp", registry);
      DoubleYoVariable startOfRampDown = new DoubleYoVariable("startOfRampDown", registry);
      DoubleYoVariable endOfRampDown = new DoubleYoVariable("endOfRampDown", registry);

      YoMiniJerkUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoMiniJerkUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      startOfRampUp.set(0.1);
      endOfRamp.set(0.3);
      startOfRampDown.set(0.5);
      endOfRampDown.set(0.7);

      double value;
      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.1);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.2);
      assertEquals(value, 0.5, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.3);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.4);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.5);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.6);
      assertEquals(value, 0.5, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.7);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.8);
      assertEquals(value, 0.0, EPSILON);


      startOfRampUp.set(0.8);
      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.1);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.2);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.3);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.4);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.5);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.6);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.7);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.8);
      assertEquals(value, 0.0, EPSILON);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testHalfWay()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      DoubleYoVariable startOfRampUp = new DoubleYoVariable("startOfRampUp", registry);
      DoubleYoVariable endOfRamp = new DoubleYoVariable("endOfRamp", registry);
      DoubleYoVariable startOfRampDown = new DoubleYoVariable("startOfRampDown", registry);
      DoubleYoVariable endOfRampDown = new DoubleYoVariable("endOfRampDown", registry);

      YoMiniJerkUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoMiniJerkUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      startOfRampUp.set(0.1);
      endOfRamp.set(0.3);
      startOfRampDown.set(0.5);
      endOfRampDown.set(0.7);

      double value;
      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.1);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.2);
      assertEquals(value, 0.5, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.3);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.4);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.5);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.6);
      assertEquals(value, 0.5, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.7);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.8);
      assertEquals(value, 0.0, EPSILON);
   }


   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSmallDifferences()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      DoubleYoVariable startOfRampUp = new DoubleYoVariable("startOfRampUp", registry);
      DoubleYoVariable endOfRamp = new DoubleYoVariable("endOfRamp", registry);
      DoubleYoVariable startOfRampDown = new DoubleYoVariable("startOfRampDown", registry);
      DoubleYoVariable endOfRampDown = new DoubleYoVariable("endOfRampDown", registry);

      YoMiniJerkUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoMiniJerkUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      startOfRampUp.set(0.1);
      endOfRamp.set(startOfRampUp.getDoubleValue() + EPSILON);
      startOfRampDown.set(0.5);
      endOfRampDown.set(startOfRampDown.getDoubleValue() + EPSILON);

      double value;
      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.1);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.1 + EPSILON/2.0);
      assertEquals(value, 0.5, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.3);
      assertEquals(value, 1.0, EPSILON);


      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.5);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.5 + EPSILON/2.0);
      assertEquals(value, 0.5, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.7);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.8);
      assertEquals(value, 0.0, EPSILON);
   }






   private void testRangeOfAlphas(double expectedValue, YoMiniJerkUpAndDownAlphaToAlpha yoMiniJerkUpAndDownAlphaToAlpha)
   {
      for(double alpha = -1.0; alpha < 2.0 ; alpha = alpha + 0.001)
      {
         double value = yoMiniJerkUpAndDownAlphaToAlpha.getAlphaPrime(alpha);
         assertEquals(value, expectedValue, EPSILON);
      }
   }
}