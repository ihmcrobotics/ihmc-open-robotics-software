package us.ihmc.robotics.alphaToAlpha;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Created by Peter on 9/9/2016.
 */
@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class YoVariableRampUpAndDownAlphaToAlphaTest
{
   private double EPSILON = 1e-6;
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInvalidYoVariables()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

      YoVariableRampUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoVariableRampUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      testRangeOfAlphas(0.0, yoVariableRampUpAndDownAlphaToAlpha);

      startOfRampUp.set(0.1);
      testRangeOfAlphas(0.0, yoVariableRampUpAndDownAlphaToAlpha);

      endOfRamp.set(0.2);
      testRangeOfAlphas(0.0, yoVariableRampUpAndDownAlphaToAlpha);

      startOfRampDown.set(0.3);
      testRangeOfAlphas(0.0, yoVariableRampUpAndDownAlphaToAlpha);

      endOfRampDown.set(1.0);
      testRangeOfAlphas(0.0, yoVariableRampUpAndDownAlphaToAlpha);

      endOfRampDown.set(0.9);
      startOfRampDown.set(0.95);
      testRangeOfAlphas(0.0, yoVariableRampUpAndDownAlphaToAlpha);
   }



   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testValidYoVariables()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

      YoVariableRampUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoVariableRampUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneQuaterOfTheWay()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

      YoVariableRampUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoVariableRampUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

      startOfRampUp.set(0.1);
      endOfRamp.set(0.3);
      startOfRampDown.set(0.5);
      endOfRampDown.set(0.7);

      double value;
      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.1);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.15);
      assertEquals(value, 0.25, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.25);
      assertEquals(value, 0.75, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.3);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.4);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.5);
      assertEquals(value, 1.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.55);
      assertEquals(value, 0.75, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.65);
      assertEquals(value, 0.25, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.7);
      assertEquals(value, 0.0, EPSILON);

      value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(0.8);
      assertEquals(value, 0.0, EPSILON);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSmallDifferences()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

      YoVariableRampUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha = new YoVariableRampUpAndDownAlphaToAlpha(startOfRampUp, endOfRamp, startOfRampDown, endOfRampDown);

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






   private void testRangeOfAlphas(double expectedValue, YoVariableRampUpAndDownAlphaToAlpha yoVariableRampUpAndDownAlphaToAlpha)
   {
      for(double alpha = -1.0; alpha < 2.0 ; alpha = alpha + 0.001)
      {
         double value = yoVariableRampUpAndDownAlphaToAlpha.getAlphaPrime(alpha);
         assertEquals(value, expectedValue, EPSILON);
      }
   }
}