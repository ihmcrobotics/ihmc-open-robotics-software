package us.ihmc.simulationConstructionSetTools.util.dataProcessors;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class ValueDataCheckerParametersTest
{
   private double EPSILON = 1e-10;
   
   @Test
      public void testGetDefensiveCopy()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();
      valueDataCheckerParametersOriginal.setErrorThresholdOnDerivativeComparison(Math.random());
      valueDataCheckerParametersOriginal.setMaximumDerivative(Math.random());
      valueDataCheckerParametersOriginal.setMaximumSecondDerivative(Math.random());
      valueDataCheckerParametersOriginal.setMaximumValue(Math.random());
      valueDataCheckerParametersOriginal.setMinimumValue(valueDataCheckerParametersOriginal.getMaximumValue() - 1.0);
      
      ValueDataCheckerParameters valueDataCheckerParametersCopy = valueDataCheckerParametersOriginal.getDefensiveCopy();
      
      assertEquals(valueDataCheckerParametersOriginal.getErrorThresholdOnDerivativeComparison(), valueDataCheckerParametersCopy.getErrorThresholdOnDerivativeComparison(), EPSILON);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumDerivative(), valueDataCheckerParametersCopy.getMaximumDerivative(), EPSILON);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumSecondDerivative(), valueDataCheckerParametersCopy.getMaximumSecondDerivative(), EPSILON);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumValue(), valueDataCheckerParametersCopy.getMaximumValue(), EPSILON);
      assertEquals(valueDataCheckerParametersOriginal.getMinimumValue(), valueDataCheckerParametersCopy.getMinimumValue(), EPSILON);
   }

   @Test
      public void testErrorThresholdOnDerivativeComparison()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = Math.random();
      valueDataCheckerParametersOriginal.setErrorThresholdOnDerivativeComparison(value);
      assertEquals(valueDataCheckerParametersOriginal.getErrorThresholdOnDerivativeComparison(), value, EPSILON);
      
      valueDataCheckerParametersOriginal.setErrorThresholdOnDerivativeComparison(-value);
      assertEquals(valueDataCheckerParametersOriginal.getErrorThresholdOnDerivativeComparison(), value, EPSILON);
   }


   @Test
      public void testMaximumDerivative()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = Math.random();
      valueDataCheckerParametersOriginal.setMaximumDerivative(value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumDerivative(), value, EPSILON);
      
      valueDataCheckerParametersOriginal.setMaximumDerivative(-value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumDerivative(), value, EPSILON);
   }


   @Test
   public void testMaximumSecondDerivative()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = Math.random();
      valueDataCheckerParametersOriginal.setMaximumSecondDerivative(value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumSecondDerivative(), value, EPSILON);
      
      valueDataCheckerParametersOriginal.setMaximumSecondDerivative(-value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumSecondDerivative(), value, EPSILON);
   }

   @Test
      public void testMaximumValue()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = Math.random();
      valueDataCheckerParametersOriginal.setMaximumValue(value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumValue(), value, EPSILON);
      
      valueDataCheckerParametersOriginal.setMaximumValue(-value);
      assertFalse(valueDataCheckerParametersOriginal.getMaximumValue() == value);
   }


   @Test
   public void testMinimumValue()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = Math.random();
      valueDataCheckerParametersOriginal.setMinimumValue(value);
      assertEquals(valueDataCheckerParametersOriginal.getMinimumValue(), value, EPSILON);
      
      valueDataCheckerParametersOriginal.setMinimumValue(-value);
      assertFalse(valueDataCheckerParametersOriginal.getMinimumValue() == value);
   }

   @Test
   public void testSetMinGreaterThanMax()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = 10.0;
      valueDataCheckerParametersOriginal.setMaximumValue(value);
      valueDataCheckerParametersOriginal.setMinimumValue(value + 1.0);
   }
   
   @Test
   public void testSetMaxLessThanMin()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      double value = 10.0;
      valueDataCheckerParametersOriginal.setMinimumValue(value);
      valueDataCheckerParametersOriginal.setMaximumValue(value - 10.0);
   }

}
