package us.ihmc.tools;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class UnitConversionsTest
{
   private static final double EPS = 1e-12;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void inchToMeter()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double inches = rand.nextFloat();
         double meter = inches * UnitConversions.INCH_TO_METER;
         assertEquals(meter, inches * 0.0254, EPS);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void squareInchToSquareMeter()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double sideAInInches = rand.nextDouble();
         double sideBInInches = rand.nextDouble();
         double areaInInches = sideAInInches * sideBInInches;

         double areaInMeters = (sideAInInches * UnitConversions.INCH_TO_METER) * (sideBInInches * UnitConversions.INCH_TO_METER);
         assertEquals(areaInInches * UnitConversions.SQUAREINCH_TO_SQUAREMETER, areaInMeters, EPS);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void cubicInchToCubicMeter()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double sideAInInches = rand.nextDouble();
         double sideBInInches = rand.nextDouble();
         double sideCInInches = rand.nextDouble();

         double volumeInInches = sideAInInches * sideBInInches * sideCInInches;

         double volumeInMeters = (sideAInInches * UnitConversions.INCH_TO_METER) * (sideBInInches * UnitConversions.INCH_TO_METER)
                                 * (sideCInInches * UnitConversions.INCH_TO_METER);
         assertEquals(volumeInInches * UnitConversions.CUBICINCH_TO_CUBICMETER, volumeInMeters, EPS);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void degreesToRadians()
   {
      Random rand = new Random();
      for (int i = 0; i < 360; i++)
      {
         double degrees = rand.nextDouble();
         assertEquals(degrees * UnitConversions.DEG_TO_RAD, Math.toRadians(degrees), EPS);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void psiToPascals()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double psi = rand.nextDouble();
         assertEquals(psi * UnitConversions.PSI_TO_PASCALS, psi * 6894.75729, EPS);
      }
   }
}
