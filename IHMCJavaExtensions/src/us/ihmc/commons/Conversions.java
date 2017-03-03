package us.ihmc.commons;

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

   // Alternatives to TimeUnit that maintain precision

   public static long microSecondsToNanoseconds(long timeInMicroSeconds)
   {
      return timeInMicroSeconds * microToNano;
   }

   public static double nanoSecondstoSeconds(long timeInNanoSeconds)
   {
      return ((double) timeInNanoSeconds) / 1e9;
   }

   public static long secondsToNanoSeconds(double timeInSeconds)
   {
      return (long) (timeInSeconds * 1e9);
   }

   public static int secondsToMilliSeconds(double timeInSeconds)
   {
      return (int) (timeInSeconds * 1000);
   }

   public static double milliSecondsToSeconds(long timeInMilliSeconds)
   {
      return (double) (timeInMilliSeconds / 1000.0);
   }

   public static double milliSecondsToMinutes(long timeInMilliSeconds)
   {
      return (double) (timeInMilliSeconds / 60000.0);
   }

   public static long milliSecondsToNanoSeconds(long timeInMilliseconds)
   {
      return ((long) timeInMilliseconds) * 1000000L;
   }

   public static double microSecondsToSeconds(long timeInMicroseconds)
   {
      return ((double) timeInMicroseconds) / 1e6;
   }

   public static double minutesToSeconds(double minutes)
   {
      return minutes * 60.0;
   }

   public static double secondsToMinutes(double seconds)
   {
      return seconds / 60.0;
   }

   public static long nanoSecondsToMillis(long nanos)
   {
      return nanos / 1000000L;
   }

   public static long nanoSecondsToMicroseconds(long nanoSeconds)
   {
      return nanoSeconds / microToNano;
   }
}
