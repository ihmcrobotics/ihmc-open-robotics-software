package us.ihmc.avatar.logProcessor;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

/**
 * Used as a scratchpad to calculate RMS error data for papers.
 */
public class LogCSVProcessor
{
   private final String LOG_PATH = Paths.get(System.getProperty("log.path")).toString();

   public LogCSVProcessor()
   {
      Path filePath = Paths.get(LOG_PATH);

      List<String> fileContents;
      try
      {
         fileContents = Files.readAllLines(filePath, StandardCharsets.UTF_8);
      }
      catch (IOException e)
      {
         throw new RuntimeException();
      }

      fileContents.remove(0); // Remove header

      TDoubleArrayList controllerICPErrorXs = new TDoubleArrayList();
      TDoubleArrayList controllerICPErrorYs = new TDoubleArrayList();

      for (String line : fileContents)
      {
         String[] splitLine = line.split(",");
         controllerICPErrorXs.add(Double.parseDouble(splitLine[0]));
         controllerICPErrorYs.add(Double.parseDouble(splitLine[1]));
      }

      LogTools.info("RMS controllerICPErrorXs: {}", calculateRMS(controllerICPErrorXs));
      LogTools.info("RMS controllerICPErrorYs: {}", calculateRMS(controllerICPErrorYs));
   }
   
   public double calculateRMS(TDoubleArrayList values)
   {
      double sum = 0.0;
      for (int i = 0; i < values.size(); i++)
      {
         double value = values.get(i);
         sum += value * value;
      }
      double averageSquare = sum / values.size();
      return Math.sqrt(averageSquare);
   }

   public static void main(String[] args)
   {
      new LogCSVProcessor();
   }
}
