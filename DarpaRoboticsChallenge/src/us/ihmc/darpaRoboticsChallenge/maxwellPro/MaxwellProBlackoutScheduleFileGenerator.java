package us.ihmc.darpaRoboticsChallenge.maxwellPro;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

import us.ihmc.darpaRoboticsChallenge.maxwellPro.blackoutGenerators.ConstantBlackoutGenerator;

public class MaxwellProBlackoutScheduleFileGenerator
{
   private BufferedWriter writer;
   private MaxwellTime time = new MaxwellTime();
   
   public MaxwellProBlackoutScheduleFileGenerator(String outputFile) throws IOException
   {
         writer = Files.newBufferedWriter(Paths.get(outputFile), StandardCharsets.UTF_8);
   }
   
   public void generateScheduleFile(int runLengthInSeconds, MaxwellProBlackoutGenerator blackoutGenerator) throws IOException
   {
      writer.write("good interval  start time,blackout  length (sec),Blackout start time\n");
      writer.flush();
      while(time.inSeconds() < runLengthInSeconds)
      {
         String output = time.toString() + ",";

         int blackoutLength = blackoutGenerator.calculateNextBlackoutLength(time.inSeconds());
         
         output += blackoutLength + ",";
         time.addSeconds(1);
         output += time.toString();
         
         writer.write(output);
         writer.newLine();
         writer.flush();
         time.addSeconds(blackoutLength);
      }
   }
   
   public static void main(String[] args)
   {
      int runLength = 3600;
      String outputFileName = "constantBlackoutSchedule.csv";
      
      MaxwellProBlackoutGenerator blackoutGenerator = new ConstantBlackoutGenerator(3);
      
      try
      {
         new MaxwellProBlackoutScheduleFileGenerator("resources/" + outputFileName).generateScheduleFile(runLength, blackoutGenerator);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
}
