package us.ihmc.robotics.alphaToAlpha;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Created by Peter on 9/11/2016.
 */
public class YoMiniJerkUpAndDownAlphaToAlphaTest
{
   private double EPSILON = 1e-6;
   @Test
   public void testInvalidYoVariables()
   {
      YoRegistry registry = new YoRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

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



   @Test
   public void testValidYoVariables()
   {
      YoRegistry registry = new YoRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

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

   @Test
   public void testHalfWay()
   {
      YoRegistry registry = new YoRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

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


   @Test
   public void testSmallDifferences()
   {
      YoRegistry registry = new YoRegistry("dummy");
      YoDouble startOfRampUp = new YoDouble("startOfRampUp", registry);
      YoDouble endOfRamp = new YoDouble("endOfRamp", registry);
      YoDouble startOfRampDown = new YoDouble("startOfRampDown", registry);
      YoDouble endOfRampDown = new YoDouble("endOfRampDown", registry);

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