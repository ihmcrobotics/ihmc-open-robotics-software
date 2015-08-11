package us.ihmc.robotics.linearDynamicSystems;

/**
 * <p>Static methods for converting from RadPerSecond to Hz, radians to degrees, and units to decibels.</p>
 */
public class BodeUnitsConverter
{
   public static double[] convertRadPerSecondToHz(double[] freqInRadPerSecond)
   {
      double[] frequencyInHz = new double[freqInRadPerSecond.length];

      for (int i = 0; i < freqInRadPerSecond.length; i++)
      {
         frequencyInHz[i] = convertRadPerSecondToHz(freqInRadPerSecond[i]);
      }

      return frequencyInHz;
   }

   public static double convertRadPerSecondToHz(double freqInRadPerSecond)
   {
      double frequencyInHz = freqInRadPerSecond / (2.0 * Math.PI);

      return frequencyInHz;
   }

   public static double[] convertRadianToDegrees(double[] phaseInRadian)
   {
      double[] phaseInDegrees = new double[phaseInRadian.length];

      for (int i = 0; i < phaseInRadian.length; i++)
      {
         phaseInDegrees[i] = convertRadianToDegrees(phaseInRadian[i]);
      }

      return phaseInDegrees;
   }

   public static double convertRadianToDegrees(double phaseInRadian)
   {
      double phaseInDegrees = phaseInRadian * 180.0 / Math.PI;

      return phaseInDegrees;
   }


   public static double[] convertMagnitudeToDecibels(double[] magnitudesInUnits)
   {
      int n = magnitudesInUnits.length;

      double[] magnitudeInDecibels = new double[n];

      for (int i = 0; i < n; i++)
      {
         magnitudeInDecibels[i] = convertMagnitudeToDecibels(magnitudesInUnits[i]);
      }

      return magnitudeInDecibels;
   }

   public static double convertMagnitudeToDecibels(double magnitudesInUnits)
   {
      double magnitudeInDecibels = 20.0 * Math.log10(magnitudesInUnits);

      return magnitudeInDecibels;
   }


}
