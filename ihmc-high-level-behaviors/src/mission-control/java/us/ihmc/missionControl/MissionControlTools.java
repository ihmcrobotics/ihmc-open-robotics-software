package us.ihmc.missionControl;

import us.ihmc.log.LogTools;
import us.ihmc.tools.processManagement.ProcessTools;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

public final class MissionControlTools
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

   /**
    * Find any service file in /etc/systemd/system that contains:
    *    `IsMissionControllable=yes`
    * Returns the list of service names that matched.
    */
   public static List<String> findSystemdServiceNames()
   {
      List<String> missionControllableServices = new ArrayList<>();

      File serviceFilesDir = new File("/etc/systemd/system");

      if (serviceFilesDir.exists())
      {
         for (File serviceFile : serviceFilesDir.listFiles())
         {
            if (serviceFile.isDirectory())
               continue;

            if (!serviceFile.getName().endsWith(".service"))
               continue;

            try
            {
               for (String line : Files.readAllLines(serviceFile.toPath()))
                  if (line.contains("MissionControl=yes"))
                     missionControllableServices.add(serviceFile.getName().replace(".service", ""));
            }
            catch (IOException e)
            {
               LogTools.info("Unable to read service file: " + serviceFile.getName());
            }
         }
      }

      return missionControllableServices;
   }

   public static String getServiceStatus(String serviceName)
   {
      String statusOutput = ProcessTools.execSimpleCommandSafe("systemctl status " + serviceName);
      String[] lines = statusOutput.split("\\R");
      if (lines.length < 3)
      {
         LogTools.error("Got: {}", statusOutput);
         return "error parsing systemd status";
      }
      else
      {
         return lines[2].trim();
      }
   }
}
