package us.ihmc.yoVariables.filters;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class RateLimitedYoVariableTest
{
   YoRegistry registry = new YoRegistry("registry");
   RateLimitedYoVariable rateLimitedYoVariable1, rateLimitedYoVariable2, rateLimitedYoVariable3, rateLimitedYoVariable4;
   YoDouble maxRate2, maxRate4;
   YoDouble position3, position4;
   double maxRate1, maxRate3;
   double dt1, dt2, dt3, dt4;

   @BeforeEach
   public void setUp()
   {
      maxRate2 = new YoDouble("maxRate2", registry);
      maxRate4 = new YoDouble("maxRate4", registry);
      position3 = new YoDouble("position3", registry);
      position4 = new YoDouble("position4", registry);

      maxRate1 = 10.0;
      maxRate2.set(9.0);
      maxRate3 = 11.0;
      maxRate4.set(12.0);

      dt1 = 1.0;
      dt2 = 1.0;
      dt3 = 1.0;
      dt4 = 1.0;

      position3.set(0.5);
      position4.set(0.75);

      rateLimitedYoVariable1 = new RateLimitedYoVariable("rateLimitedYoVariable1", registry, maxRate1, dt1);
      rateLimitedYoVariable2 = new RateLimitedYoVariable("rateLimitedYoVariable2", registry, maxRate2, dt2);
      rateLimitedYoVariable3 = new RateLimitedYoVariable("rateLimitedYoVariable3", registry, maxRate3, position3, dt3);
      rateLimitedYoVariable4 = new RateLimitedYoVariable("rateLimitedYoVariable4", registry, maxRate4, position4, dt4);
   }

	@Test
   public void testUpdate()
   {
      try
      {
         rateLimitedYoVariable3.update();
         rateLimitedYoVariable4.update();
      }
      catch (Exception e)
      {
         fail();
      }
   }

	@Test
   public void testUpdateWithNullPointerException()
   {
      try
      {
         rateLimitedYoVariable1.update();
         rateLimitedYoVariable2.update();

         fail("Did not throw NullPointerException.");
      }
      catch (Exception e)
      {

      }
   }

	@Test
   public void testUpdateWithCurrentPositionParameter()
   {
      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition1 = 10.0 * Math.sin(angle);
         rateLimitedYoVariable1.update(currentPosition1);
         assertEquals(rateLimitedYoVariable1.getDoubleValue(), currentPosition1, 1E-13);
      }

      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition2 = 7.0 * Math.sin(angle);
         rateLimitedYoVariable2.update(currentPosition2);
         assertEquals(rateLimitedYoVariable2.getDoubleValue(), currentPosition2, 1E-13);
      }

      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition3 = 11.0 * Math.sin(angle);
         rateLimitedYoVariable3.update(currentPosition3);
         assertEquals(rateLimitedYoVariable3.getDoubleValue(), currentPosition3, 1E-13);
      }

      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition4 = 12.0 * Math.sin(angle);
         rateLimitedYoVariable4.update(currentPosition4);
         assertEquals(rateLimitedYoVariable4.getDoubleValue(), currentPosition4, 1E-13);
      }
   }

	@Test
   public void testUpdateWithCurrentPositionParameterExceedingMaxRate()
   {
      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition1 = 25.0 * Math.sin(angle);
         double dSinTheta = Math.cos(angle);
         double signOfSlope = Math.signum(dSinTheta) * 1.0;

         if (Math.abs(currentPosition1 - rateLimitedYoVariable1.getDoubleValue()) > maxRate1)
         {
            currentPosition1 = rateLimitedYoVariable1.getDoubleValue() + signOfSlope * maxRate1;
         }
         rateLimitedYoVariable1.update(currentPosition1);
         assertEquals(rateLimitedYoVariable1.getDoubleValue(), currentPosition1, 1E-15);
      }

      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition2 = 25.0 * Math.sin(angle);
         double dSinTheta = Math.cos(angle);
         double signOfSlope = Math.signum(dSinTheta) * 1.0;

         if (Math.abs(currentPosition2 - rateLimitedYoVariable2.getDoubleValue()) > maxRate2.getDoubleValue())
         {
            currentPosition2 = rateLimitedYoVariable2.getDoubleValue() + signOfSlope * maxRate2.getDoubleValue();
         }
         rateLimitedYoVariable2.update(currentPosition2);
         assertEquals(rateLimitedYoVariable2.getDoubleValue(), currentPosition2, 1E-15);
      }

      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition3 = 25.0 * Math.sin(angle);
         double dSinTheta = Math.cos(angle);
         double signOfSlope = Math.signum(dSinTheta) * 1.0;

         if (Math.abs(currentPosition3 - rateLimitedYoVariable3.getDoubleValue()) > maxRate3)
         {
            currentPosition3 = rateLimitedYoVariable3.getDoubleValue() + signOfSlope * maxRate3;
         }
         rateLimitedYoVariable3.update(currentPosition3);
         assertEquals(rateLimitedYoVariable3.getDoubleValue(), currentPosition3, 1E-15);
      }

      for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
      {
         double currentPosition4 = 25.0 * Math.sin(angle);
         double dSinTheta = Math.cos(angle);
         double signOfSlope = Math.signum(dSinTheta) * 1.0;

         if (Math.abs(currentPosition4 - rateLimitedYoVariable4.getDoubleValue()) > maxRate4.getDoubleValue())
         {
            currentPosition4 = rateLimitedYoVariable4.getDoubleValue() + signOfSlope * maxRate4.getDoubleValue();
         }
         rateLimitedYoVariable4.update(currentPosition4);
         assertEquals(rateLimitedYoVariable4.getDoubleValue(), currentPosition4, 1E-15);
      }
   }

	@Test
   public void testUpdateWithMaxRateBeingNegative()
   {
      try
      {
         RateLimitedYoVariable rateLimitedYoVariableWithNegativeMaxRate = new RateLimitedYoVariable("rateLimitedYoVariableWithNegativeMaxRate", registry, -5.0,
               1.0);
         rateLimitedYoVariableWithNegativeMaxRate.update(5.0);
      }
      catch (RuntimeException e)
      {
         assertEquals(e.getMessage(), "The maxRate parameter in the RateLimitedYoVariable cannot be negative.");
      }
   }
}