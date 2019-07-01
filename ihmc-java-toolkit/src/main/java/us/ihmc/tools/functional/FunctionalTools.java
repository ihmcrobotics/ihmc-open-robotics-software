package us.ihmc.tools.functional;

public class FunctionalTools
{
   public static void runIfTrue(boolean condition, Runnable runnable)
   {
      if (condition)
      {
         runnable.run();
      }
   }
}
