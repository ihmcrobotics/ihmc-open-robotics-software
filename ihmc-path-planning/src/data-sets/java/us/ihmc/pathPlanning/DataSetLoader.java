package us.ihmc.pathPlanning;

import org.apache.commons.io.IOUtils;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

import static java.nio.charset.StandardCharsets.UTF_8;

public class DataSetLoader
{
   static final String DATA_SET_DIRECTORY_PATH = "us/ihmc/pathPlanning/dataSets";
   private static final String DATA_SET_LIST_FILENAME = "DataSetList.txt";
   private static final String PLANAR_REGIONS_DIRECTORY = "PlanarRegions";
   private static final String PLANNER_INPUTS_FILENAME = "PlannerInputs.txt";

   public static List<DataSet> loadDataSets()
   {
      Class<DataSetLoader> loadingClass = DataSetLoader.class;
      InputStream dataSetList = loadingClass.getResourceAsStream(DATA_SET_LIST_FILENAME);
      List<String> dataSetNamesList;

      try
      {
         dataSetNamesList = IOUtils.readLines(dataSetList, UTF_8);
      }
      catch(IOException e)
      {
         throw new RuntimeException("Unable to read dataset names list. expected filename: " + DATA_SET_LIST_FILENAME);
      }

      ArrayList<DataSet> dataSets = new ArrayList<>();
      for (int i = 0; i < dataSetNamesList.size(); i++)
      {
         String dataSetName = dataSetNamesList.get(i);
         String dataSetPlanarRegionsPath = DATA_SET_DIRECTORY_PATH + "/" + dataSetName + "/" + PLANAR_REGIONS_DIRECTORY;
         PlanarRegionsList planarRegionsList = PlanarRegionFileTools.importPlanarRegionData(loadingClass.getClassLoader(), dataSetPlanarRegionsPath);

         DataSet dataSet = new DataSet(dataSetName, planarRegionsList);
         InputStream plannerInputsStream = loadingClass.getClassLoader()
                                                    .getResourceAsStream(DATA_SET_DIRECTORY_PATH + "/" + dataSetName + "/" + PLANNER_INPUTS_FILENAME);
         try
         {
            loadPlannerInputs(plannerInputsStream, dataSet);
         }
         catch(IOException e)
         {
            System.err.println("Unable to read planner inputs for dataset: " + dataSetName + ". Skipping dataset");
            continue;
         }

         dataSets.add(dataSet);
      }

      return dataSets;
   }

   private static void loadPlannerInputs(InputStream inputStream, DataSet dataSet) throws IOException
   {
      BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream, UTF_8));

      String line;
      while((line = reader.readLine()) != null)
      {
         if(line.equals(""))
            continue;

         String[] lineSubstrings = line.split(" ");
         switch (lineSubstrings[0])
         {
         case "startPosition":
         {
            dataSet.setStartPosition(Double.parseDouble(lineSubstrings[1]), Double.parseDouble(lineSubstrings[2]), Double.parseDouble(lineSubstrings[3]));
            break;
         }
         case "goalPosition":
         {
            dataSet.setGoalPosition(Double.parseDouble(lineSubstrings[1]), Double.parseDouble(lineSubstrings[2]), Double.parseDouble(lineSubstrings[3]));
            break;
         }
         case "startYaw":
         {
            dataSet.setStartYaw(Double.parseDouble(lineSubstrings[1]));
            break;
         }
         case "goalYaw":
         {
            dataSet.setGoalYaw(Double.parseDouble(lineSubstrings[1]));
            break;
         }
         default:
         {
            if(lineSubstrings.length > 1)
            {
               for (int i = 1; i < lineSubstrings.length; i++)
               {
                  dataSet.addAdditionalData(lineSubstrings[0], lineSubstrings[i]);
               }
            }
            break;
         }
         }
      }
   }
}
