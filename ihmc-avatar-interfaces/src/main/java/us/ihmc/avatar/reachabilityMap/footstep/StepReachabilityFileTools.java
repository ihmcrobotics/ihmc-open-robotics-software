package us.ihmc.avatar.reachabilityMap.footstep;

import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;

import java.io.*;
import java.util.Map;
import java.util.Scanner;

public class StepReachabilityFileTools
{
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private static final String filePath = "ihmc-open-robotics-software/valkyrie/src/main/java/us/ihmc/valkyrie/log/";

   public static void writeToFile(Map<StepReachabilityLatticePoint, Double> feasibilityMap, double spacingXY, int yawDivisions, double yawSpacing)
   {
      FileWriter fileWriter;
      String reachabilityDataFileName = filePath + "StepReachabilityMap.txt";

      try
      {
         File reachabilityDataFile = new File(reachabilityDataFileName);
         FileTools.ensureFileExists(reachabilityDataFile.toPath());
         fileWriter = new FileWriter(reachabilityDataFile);

         fileWriter.write(spacingXY + ", ");
         fileWriter.write(yawDivisions + ", ");
         fileWriter.write(yawSpacing + "\n");

         for (StepReachabilityLatticePoint latticePoint : feasibilityMap.keySet())
         {
            fileWriter.write(latticePoint.getXIndex() + ", ");
            fileWriter.write(latticePoint.getYIndex() + ", ");
            fileWriter.write(latticePoint.getYawIndex() + ", ");
            fileWriter.write(String.valueOf(feasibilityMap.get(latticePoint)));
            fileWriter.write("\n");
         }

         fileWriter.flush();
         LogTools.info("Done writing to file");
      }
      catch (Exception e)
      {
         LogTools.error("Error logging reachability file");
         e.printStackTrace();
      }
   }

   public static void printFeasibilityMap(StepReachabilityData stepReachabilityData)
   {
      for (StepReachabilityLatticePoint latticePoint : stepReachabilityData.getLegReachabilityMap().keySet())
      {
         System.out.print(latticePoint);
      }
   }

   public static StepReachabilityData loadFromFile(String filename)
   {
      String reachabilityDataFileName = filePath + filename;
      StepReachabilityData reachabilityData = new StepReachabilityData();

      try
      {
         Scanner scanner = new Scanner(new File(reachabilityDataFileName));

         // Read grid spacing
         String gridData = scanner.nextLine();
         String[] gridDataStrings = gridData.split(",");
         double spacingXY = Double.parseDouble(gridDataStrings[0]);
         int yawDivisions = Integer.parseInt(gridDataStrings[1]);
         double yawSpacing = Double.parseDouble(gridDataStrings[2]);
         reachabilityData.setGridData(spacingXY, yawSpacing, yawDivisions);

         while(scanner.hasNextLine())
         {
            String line = scanner.nextLine();

            // Parse to get frame position, orientation and feasibility boolean
            String[] data = line.split(",");
            int xIndex = Integer.parseInt(data[0]);
            int yIndex = Integer.parseInt(data[1]);
            int yawIndex = Integer.parseInt(data[2]);
            double reachabilityValue = Double.parseDouble(data[3]);
            StepReachabilityLatticePoint latticePoint = new StepReachabilityLatticePoint(xIndex, yIndex, yawIndex);

            reachabilityData.getLegReachabilityMap().put(latticePoint, reachabilityValue);
         }
         scanner.close();
         LogTools.info("Done loading from file");
         return reachabilityData;
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      return null;
   }

}
