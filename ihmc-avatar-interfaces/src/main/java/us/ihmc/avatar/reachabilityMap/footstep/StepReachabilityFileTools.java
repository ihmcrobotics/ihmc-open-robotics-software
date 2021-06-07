package us.ihmc.avatar.reachabilityMap.footstep;

import org.lwjgl.Sys;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.ihmcPerception.linemod.LineModTemplate;
import us.ihmc.log.LogTools;

import java.io.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class StepReachabilityFileTools
{
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private static final String filePath = "ihmc-open-robotics-software/valkyrie/src/main/java/us/ihmc/valkyrie/log/";

   public static void writeToFile(Map<FramePose3D, Double> feasibilityMap)
   {
      FileWriter fileWriter;
      String reachabilityDataFileName = filePath + "StepReachabilityMap.txt";

      try
      {
         File reachabilityDataFile = new File(reachabilityDataFileName);
         FileTools.ensureFileExists(reachabilityDataFile.toPath());
         fileWriter = new FileWriter(reachabilityDataFile);

         for (FramePose3D frame : feasibilityMap.keySet())
         {
            String position = frame.toString().substring(21,44);
            String orientation = frame.toString().substring(62,92);
            fileWriter.write(position + ", ");
            fileWriter.write(orientation + ", ");
            fileWriter.write(String.valueOf(feasibilityMap.get(frame)));
            fileWriter.write("\n");
         }

         fileWriter.flush();
         System.out.println("Done writing to file");
      }
      catch (Exception e)
      {
         LogTools.error("Error logging reachability file");
         e.printStackTrace();
      }
   }

   public static void printFeasibilityMap(Map<FramePose3D, Double> feasibilityMap)
   {
      for (FramePose3D frame : feasibilityMap.keySet())
      {
         System.out.print("Frame: " + frame);
         System.out.println(", Feasiblility: " + feasibilityMap.get(frame));
      }
   }

   public static Map<FramePose3D, Double> loadFromFile(String filename)
   {
      String reachabilityDataFileName = filePath + filename;
      Map<FramePose3D, Double> feasibilityMap = new HashMap<>();

      try
      {
         Scanner scanner = new Scanner(new File(reachabilityDataFileName));
         while(scanner.hasNextLine())
         {
            String line = scanner.nextLine();
            FramePose3D frame = new FramePose3D();

            // Parse to get frame position, orientation and feasibility boolean
            String[] data = line.split(",");
            double posX = Double.parseDouble(data[0]);
            double posY = Double.parseDouble(data[1]);
            double posZ = Double.parseDouble(data[2]);
            frame.getPosition().set(posX, posY, posZ);

            double orX = Double.parseDouble(data[3]);
            double orY = Double.parseDouble(data[4]);
            double orZ = Double.parseDouble(data[5]);
            double orS = Double.parseDouble(data[6]);
            frame.getOrientation().set(orX, orY, orZ, orS);

            double reachabilityValue = Double.parseDouble(data[7]);
            feasibilityMap.put(frame, reachabilityValue);
         }
         scanner.close();
         System.out.println("Done loading from file");
         return feasibilityMap;
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      return null;
   }

}
