package us.ihmc.resourceMonitor;

import us.ihmc.tools.processManagement.ProcessTools;

import java.io.IOException;

public final class ResourceMonitorTools
{
   public static boolean sysstatAvailable()
   {
      try
      {
         ProcessTools.execSimpleCommand("sar");
      }
      catch (IOException | InterruptedException ignored)
      {
         return false;
      }
      return true;
   }

   public static boolean nvidiaGPUAvailable()
   {
      try
      {
         ProcessTools.execSimpleCommand("nvidia-smi");
      }
      catch (IOException | InterruptedException ignored)
      {
         return false;
      }
      return true;
   }

   public static boolean lmSensorsAvailable()
   {
      try
      {
         ProcessTools.execSimpleCommand("sensors");
      }
      catch (IOException | InterruptedException ignored)
      {
         return false;
      }
      return true;
   }
}
