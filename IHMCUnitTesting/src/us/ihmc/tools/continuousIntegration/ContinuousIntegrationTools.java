package us.ihmc.tools.continuousIntegration;

public class ContinuousIntegrationTools
{
   public static boolean isRunningOnContinuousIntegrationServer()
   {
      String isBamboo = System.getenv("IS_BAMBOO");
      
      if (isBamboo == null || !isBamboo.equals("true"))
      {
         return false;
      }
      else
      {
         return true;
      }
   }
}
