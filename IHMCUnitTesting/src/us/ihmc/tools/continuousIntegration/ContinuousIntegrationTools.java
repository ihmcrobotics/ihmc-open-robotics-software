package us.ihmc.tools.continuousIntegration;

public class ContinuousIntegrationTools
{
   public static boolean isRunningOnContinuousIntegrationServer()
   {
      String runningOnContinuousIntegrationServer = System.getenv("RUNNING_ON_CONTINUOUS_INTEGRATION_SERVER");

      if (runningOnContinuousIntegrationServer == null || !runningOnContinuousIntegrationServer.equals("true"))
      {
         return false;
      }
      else
      {
         return true;
      }
   }
}
