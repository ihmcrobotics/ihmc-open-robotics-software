package us.ihmc.missionControl;

import org.apache.commons.io.IOUtils;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class MissionControlService
{
   private final SystemResourceMonitor systemResourceMonitor = new SystemResourceMonitor();

   public MissionControlService()
   {
      ExceptionHandlingThreadScheduler updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate",
                                                                                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      updateThreadScheduler.schedule(this::update, 1.0);

//      ROS2



   }

   private void update()
   {
      systemResourceMonitor.update();



//      String usedRAM = execSimpleCommand("/bin/bash -c \"free --kilo | awk 'NR==2 {print $3}'\"");
//      String totalRAM = execSimpleCommand("/bin/bash -c \"free --kilo | awk 'NR==2 {print $2}'\"");
//      String usedRAM = execSimpleCommand(new String[] {"free", "--kilo", "|", "awk 'NR==2 {print $3}'" });
//      String usedRAM = execSimpleCommand(new String[] {"free", "--kilo", "|", "awk", "'NR==2 {print $3}'"});
      String free = execSimpleCommand("free --mebi");
      String[] lines = free.split("\\R");
      String[] amongSpaces = lines[1].split("\\s+");
      double totalRAM = Double.valueOf(amongSpaces[1]) / 1000.0;
      double usedRAM = Double.valueOf(amongSpaces[2]) / 1000.0;
      LogTools.info("RAM: " + FormattingTools.getFormattedDecimal1D(usedRAM) + " / " + FormattingTools.getFormattedDecimal1D(totalRAM) + " GiB");

//      LogTools.info("RAM: " + systemResourceMonitor.getRamUsageString());
      LogTools.warn("CPU: " + systemResourceMonitor.getSystemCPUPercentageString());
      LogTools.error("Test message");
   }

   public static String execSimpleCommand(String command)
   {
      Runtime runtime = Runtime.getRuntime();
      try
      {
         Process process = runtime.exec(command);
         BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(process.getInputStream()));

         String output = "";
         int read;
         while ((read = bufferedReader.read()) > -1)
         {
            output += (char) read;
         }
//         while ((singleLine = bufferedReader.readLine()) != null)

//         new Thread(new Runnable() {
//            public void run() {
//               BufferedReader input = new BufferedReader(new InputStreamReader(process.getInputStream()));
//               String line = null;
//
//               try {
//                  while ((line = input.readLine()) != null)
//                     System.out.println(line);
//               } catch (IOException e) {
//                  e.printStackTrace();
//               }
//            }
//         }).start();
//
//         String singleLine = "";
////         while (!singleLine.isEmpty())
////         {
////            singleLine += bufferedReader.readLine();
////         }
//
//
//
//         ThreadTools.sleep(10);
//
//
////         bufferedReader.
//
//         int read;
//         while ((read = bufferedReader.read()) > -1)
//         {
//            singleLine += (char) read;
//         }

         process.waitFor();


         return output;
      }
      catch (IOException |InterruptedException e)
      {
         e.printStackTrace();
      }
      return null;
   }

   public static void main(String[] args)
   {

      new MissionControlService();
   }
}
