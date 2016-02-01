package us.ihmc.utilities.ros;

/**
 * Created by dstephen on 11/13/13.
 */
public class RosGlobalSpinMonitor
{
   private static boolean isSpinning = false;

   private RosGlobalSpinMonitor()
   {
   }

   public static synchronized void notifySpinMonitor()
   {
      isSpinning = true;
   }

   public static synchronized boolean isGlobalSpinnerSpinning()
   {
      return isSpinning;
   }
}
