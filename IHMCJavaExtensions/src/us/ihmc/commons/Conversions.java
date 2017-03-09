package us.ihmc.commons;

import java.util.concurrent.TimeUnit;

/**
 * Common conversions useful for making use of some libraries easier to read.
 */
public class Conversions
{
   /** Number of bytes (B) in a kibibyte (KiB) */
   public static final int KIBIBYTES_TO_BYTES = 1024;

   /** Number of bytes (B) in a kilobyte (KB) */
   public static final int KILOBYTES_TO_BYTES = 1000;

   private static long micro = 1000000;

   public static long nano = 1000000000;

   private static long microToNano = nano / micro;

   private Conversions()
   {
      // Disallow construction
   }

   /**
    * Convert kibibytes (KiB) to bytes (B).
    * 
    * @param kibibytes number of kibibytes
    * @return bytes number of bytes
    */
   public static int kibibytesToBytes(int kibibytes)
   {
      return kibibytes * KIBIBYTES_TO_BYTES;
   }

   /**
    * Convert kilobytes (KB) to bytes (B).
    * 
    * @param kilobytes number of kilobytes
    * @return bytes number of bytes
    */
   public static int kilobytesToBytes(int kilobytes)
   {
      return kilobytes * KILOBYTES_TO_BYTES;
   }

   /**
    * Convert mebibytes (MiB) to bytes (B).
    * 
    * @param mebibytes number of mebibytes
    * @return bytes number of bytes
    */
   public static int mebibytesToBytes(int mebibytes)
   {
      return mebibytes * (int) Math.pow(KIBIBYTES_TO_BYTES, 2);
   }

   /**
    * Convert megabytes (MB) to bytes (B).
    * 
    * @param megabytes number of megabytes
    * @return bytes number of bytes
    */
   public static int megabytesToBytes(int megabytes)
   {
      return megabytes * (int) Math.pow(KILOBYTES_TO_BYTES, 2);
   }

   /**
    * <p>Convert minutes to seconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param minutes time in minutes
    * @return Time in seconds.
    */
   public static double minutesToSeconds(double minutes)
   {
      return minutes * 60.0;
   }

   /**
    * <p>Convert seconds to minutes without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param seconds time in seconds
    * @return Time in minutes.
    */
   public static double secondsToMinutes(double seconds)
   {
      return seconds / 60.0;
   }

   /**
    * <p>Convert seconds to milliseconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param seconds time in seconds
    * @return Time in milliseconds.
    */
   public static int secondsToMilliseconds(double seconds)
   {
      return (int) (seconds * 1000);
   }

   /**
     * <p>Convert seconds to nanoseconds without losing precision.</p>
     * 
     * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
     * 
     * @param seconds time in seconds
     * @return Time in nanoseconds.
     */
   public static long secondsToNanoseconds(double seconds)
   {
      return (long) (seconds * 1e9);
   }

   /**
    * <p>Convert milliseconds to minutes without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param milliseconds time in milliseconds
    * @return Time in minutes.
    */
   public static double millisecondsToMinutes(long milliseconds)
   {
      return (double) (milliseconds / 60000.0);
   }

   /**
    * <p>Convert milliseconds to seconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param milliseconds time in milliseconds
    * @return Time in seconds.
    */
   public static double millisecondsToSeconds(long milliseconds)
   {
      return (double) (milliseconds / 1000.0);
   }

   /**
    * <p>Convert milliseconds to nanoseconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param timeInMilliSeconds time in milliseconds
    * @return Time in nanoseconds.
    */
   public static long millisecondsToNanoseconds(long milliseconds)
   {
      return ((long) milliseconds) * 1000000L;
   }

   /**
    * <p>Convert microseconds to seconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param microseconds time in microseconds
    * @return Time in seconds.
    */
   public static double microsecondsToSeconds(long microseconds)
   {
      return ((double) microseconds) / 1e6;
   }

   /**
    * <p>Convert microseconds to nanoseconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param microseconds time in microseconds
    * @return Time in nanoseconds.
    */
   public static long microsecondsToNanoseconds(long microseconds)
   {
      return microseconds * microToNano;
   }

  /**
    * <p>Convert nanoseconds to seconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param nanoseconds time in nanoseconds
    * @return Time in seconds.
    */
    public static double nanosecondsToSeconds(long nanoseconds)
   {
      return ((double) nanoseconds) / 1e9;
   }

    /**
    * <p>Convert nanoseconds to milliseconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param nanoseconds time in nanoseconds
    * @return Time in milliseconds.
    */
   public static long nanosecondsToMilliseconds(long nanoseconds)
   {
      return nanoseconds / 1000000L;
   }

   /**
    * <p>Convert nanoseconds to microseconds without losing precision.</p>
    * 
    * <p>NOTE: These methods exist as an alternative to {@link TimeUnit} but that maintain precision.</p>
    * 
    * @param nanoseconds time in nanoseconds
    * @return Time in microseconds.
    */
   public static long nanosecondsToMicroseconds(long nanoseconds)
   {
      return nanoseconds / microToNano;
   }

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
