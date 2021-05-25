package us.ihmc.valkyrie.stepReachability;

import org.lwjgl.Sys;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.ihmcPerception.linemod.LineModTemplate;
import us.ihmc.log.LogTools;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class StepReachabilityFileTools
{
   // for references on writing to file, check out
   // FootstepPlannerLogger line 245
   // AtlasMultiDataExporter.writeSpreadsheetFormattedData

   static String filePath = "ihmc-open-robotics-software/valkyrie/src/main/java/us/ihmc/valkyrie/stepReachability/";

   public static void writeToFile(Map<FramePose3D, Boolean> feasibilityMap)
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
            fileWriter.write("Frame: " + frame.toString().substring(9));
            fileWriter.write(", Feasiblility: " + feasibilityMap.get(frame));
            fileWriter.write("\n");
         }

         fileWriter.flush();
         System.out.println("Done writing to file");
      }
      catch (Exception e)
      {
         LogTools.error("Error logging reachability file");
         fileWriter = null;
         e.printStackTrace();
      }
   }

   public static void printFeasibilityMap(Map<FramePose3D, Boolean> feasibilityMap)
   {
      for (FramePose3D frame : feasibilityMap.keySet())
      {
         System.out.print("Frame: " + frame.toString().substring(9));
         System.out.println(", Feasiblility: " + feasibilityMap.get(frame));
      }
   }

//   public static Map<FramePose3D, Boolean> loadFromFile(String fileName)
//   {
//      Map<FramePose3D, Boolean> feasibilityMap = new HashMap<>();
//
//
////         String reachabilityDataFileName = filePath + "StepReachabilityMap.txt";
////         File file = new File(reachabilityDataFileName);
//
//      File file = new File("StepReachabilityMap.txt");
//      try
//      {
//         BufferedReader bufferedReader = new BufferedReader(new FileReader(file));
//         String line;
//         FramePose3D frame = new FramePose3D();
//         while((line = bufferedReader.readLine())!=null)
//         {
//            // Parse to get frame and feasibility boolean, load into Hashmap
//            String[] data = line.split(",");
//
//            double posX = Double.parseDouble(data[0]);
//            double posY = Double.parseDouble(data[1]);
//            double posZ = Double.parseDouble(data[2]);
//            frame.getPosition().set(posX, posY, posZ);
//
//            double orX = Double.parseDouble(data[3]);
//            double orY = Double.parseDouble(data[4]);
//            double orZ = Double.parseDouble(data[5]);
//            double orS = Double.parseDouble(data[6]);
//            frame.getOrientation().set(orX, orY, orZ, orS);
//
//            if (line.contains("True"))
//               feasibilityMap.put(frame, true);
//            else if (line.contains("False"))
//               feasibilityMap.put(frame, false);
//            else
//               throw new RuntimeException("Error in reachability file");
//         }
//      }
//      catch (Exception e)
//      {
//         LogTools.error("Error loading reachability file");
//         e.printStackTrace();
//      }
//      return null;
//   }

   public static void load(String filename)
   {
      String reachabilityDataFileName = filePath + filename;
      File file = new File(reachabilityDataFileName);

      load(file);
   }

   public static void load(File file)
   {
      try
      {
         BufferedReader reader = new BufferedReader(new FileReader(file));
         load(reader);
      } catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }

   }

   private static Map<FramePose3D, Boolean> load(BufferedReader reader)
   {
      try
      {
         Map<FramePose3D, Boolean> feasibilityMap = new HashMap<>();
         FramePose3D frame = new FramePose3D();
         String line;
         while((line = reader.readLine())!=null)
         {
            // Parse to get frame and feasibility boolean, load into Hashmap
            String[] data = line.split(",");

            // TODO: Fix parsing double
            double posX = Double.parseDouble(data[0]);
            double posY = Double.parseDouble(data[1]);
            double posZ = Double.parseDouble(data[2]);
            frame.getPosition().set(posX, posY, posZ);

            double orX = Double.parseDouble(data[3]);
            double orY = Double.parseDouble(data[4]);
            double orZ = Double.parseDouble(data[5]);
            double orS = Double.parseDouble(data[6]);
            frame.getOrientation().set(orX, orY, orZ, orS);

            if (line.contains("True"))
               feasibilityMap.put(frame, true);
            else if (line.contains("False"))
               feasibilityMap.put(frame, false);
            else
               throw new RuntimeException("Error in reachability file");
         }
         return feasibilityMap;

      } catch (IOException e)
      {
         LogTools.error("Error loading reachability file");
         e.printStackTrace();
      }
      return null;
   }

}
