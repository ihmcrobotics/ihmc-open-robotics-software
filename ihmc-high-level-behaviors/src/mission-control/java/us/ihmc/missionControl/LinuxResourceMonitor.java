package us.ihmc.missionControl;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class LinuxResourceMonitor
{
   private double totalRAMGiB;
   private double usedRAMGiB;

   public LinuxResourceMonitor()
   {
   }

   public void update()
   {
      String free = execSimpleCommand("free --mebi");
      String[] lines = free.split("\\R");
      String[] amongSpaces = lines[1].split("\\s+");
      totalRAMGiB = Double.parseDouble(amongSpaces[1]) / 1000.0;
      usedRAMGiB = Double.parseDouble(amongSpaces[2]) / 1000.0;
   }

   public static String execSimpleCommand(String command)
   {
      Runtime runtime = Runtime.getRuntime();
      try
      {
         Process process = runtime.exec(command);
         BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(process.getInputStream()));

         StringBuilder output = new StringBuilder();
         int read;
         while ((read = bufferedReader.read()) > -1)
         {
            output.append((char) read);
         }

         process.waitFor();

         return output.toString();
      }
      catch (IOException |InterruptedException e)
      {
         e.printStackTrace();
      }
      return null;
   }

   public double getTotalRAMGiB()
   {
      return totalRAMGiB;
   }

   public double getUsedRAMGiB()
   {
      return usedRAMGiB;
   }
}
