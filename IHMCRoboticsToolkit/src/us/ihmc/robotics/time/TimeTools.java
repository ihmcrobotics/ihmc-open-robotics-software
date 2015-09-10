package us.ihmc.robotics.time;

/**
 * Note that TimeUnit.[UNIT].convert(period, TimeUnit) exists as well.
 * 
 * @author jesper, dcalvert
 */
public class TimeTools
{
   private static long micro = 1000000;
   public static long nano = 1000000000;
  
   private static long microToNano = nano/micro;
   
   public static long microSecondsToNanoseconds(long timeInMicroSeconds)
   {
      return timeInMicroSeconds * microToNano;
   }
   
   public static double nanoSecondstoSeconds(long timeInNanoSeconds)
   {
      return ((double) timeInNanoSeconds)/1e9;
   }
   
   public static long secondsToNanoSeconds(double timeInSeconds)
   {
      return (long)(timeInSeconds * 1e9);
   }
   
   public static int secondsToMilliSeconds(double timeInSeconds) 
   {
      return (int)(timeInSeconds * 1000);
   }
   
   public static double milliSecondsToSeconds(long timeInMilliSeconds)
   {
      return (double)(timeInMilliSeconds / 1000.0);
   }
   
   public static double milliSecondsToMinutes(long timeInMilliSeconds)
   {
      return (double)(timeInMilliSeconds / 60000.0);
   }

   public static long milliSecondsToNanoSeconds(int timeInMilliseconds)
   {
      return ((long) timeInMilliseconds) * 1000000L;
   }

   public static double microSecondsToSeconds(long timeInMicroseconds)
   {
      return ((double) timeInMicroseconds)/1e6;
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
