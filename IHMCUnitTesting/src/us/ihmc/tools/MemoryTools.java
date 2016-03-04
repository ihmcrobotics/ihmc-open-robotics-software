package us.ihmc.tools;

public class MemoryTools
{
   public static int getCurrentMemoryUsageInMB()
   {
      return printCurrentMemoryUsageAndReturnUsedMemoryInMB("", false, true);
   }
   
   public static int printCurrentMemoryUsageAndReturnUsedMemoryInMB(String messagePrefix)
   {
      return printCurrentMemoryUsageAndReturnUsedMemoryInMB(messagePrefix, true, true);
   }
   
   public static int printCurrentMemoryUsageAndReturnUsedMemoryInMBWithoutGarbageCollecting(String messagePrefix)
   {
      return printCurrentMemoryUsageAndReturnUsedMemoryInMB(messagePrefix, true, false);
   }
   
   public static int printCurrentMemoryUsageAndReturnUsedMemoryInMB(String messagePrefix, boolean printResults, boolean garbageCollect)
   {
      Runtime runtime = Runtime.getRuntime();

      if (garbageCollect)
      {
         System.gc();
         System.runFinalization();
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
         }
      }
      
      long freeMemory = runtime.freeMemory();
      long totalMemory = runtime.totalMemory();
      long usedMemory = totalMemory - freeMemory;

      int usedMemoryMB = (int) (usedMemory / 1000000);
      
      if (printResults) System.out.println(messagePrefix + "freeMemory = " + freeMemory / 1000000 + "MB, totalMemory = " + totalMemory / 1000000 + "MB, usedMemory = " + usedMemoryMB + "MB");

      return usedMemoryMB;
   }
}
