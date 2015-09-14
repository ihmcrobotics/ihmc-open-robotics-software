package us.ihmc.tools.processManagement;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import org.apache.commons.lang3.SystemUtils;

public class ProcessTools
{
   public static ArrayList<String> getAllSystemProcesses()
   {
      try
      {
         Process psSystemCommand = null;

         if (SystemUtils.IS_OS_LINUX || SystemUtils.IS_OS_MAC)
            psSystemCommand = Runtime.getRuntime().exec("ps -e");
         else if (SystemUtils.IS_OS_WINDOWS)
            psSystemCommand = Runtime.getRuntime().exec(System.getenv("windir") + "\\system32\\" + "tasklist.exe");

         BufferedReader reader = new BufferedReader(new InputStreamReader(psSystemCommand.getInputStream()));

         String line;
         ArrayList<String> processLines = new ArrayList<>();
         while ((line = reader.readLine()) != null)
            processLines.add(line);
         
         return processLines;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         
         return null;
      }
   }
}
