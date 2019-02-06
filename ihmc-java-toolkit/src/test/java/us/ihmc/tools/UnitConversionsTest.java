package us.ihmc.tools;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class UnitConversionsTest
{
   private static final double EPS = 1e-12;

   @Test
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

   @Test
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

   @Test
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

   @Test
   public void degreesToRadians()
   {
      Random rand = new Random();
      for (int i = 0; i < 360; i++)
      {
         double degrees = rand.nextDouble();
         assertEquals(degrees * UnitConversions.DEG_TO_RAD, Math.toRadians(degrees), EPS);
      }
   }

   @Test
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
