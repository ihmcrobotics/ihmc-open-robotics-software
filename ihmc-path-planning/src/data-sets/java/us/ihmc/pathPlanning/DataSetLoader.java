package us.ihmc.pathPlanning;

import org.apache.commons.io.IOUtils;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import static java.nio.charset.StandardCharsets.UTF_8;

public class DataSetLoader
{
   static final String DATA_SET_DIRECTORY_PATH = "us/ihmc/pathPlanning/dataSets";
   private static final String DATA_SET_LIST_FILENAME = "DataSetList.txt";
   private static final String PLANAR_REGIONS_DIRECTORY = "PlanarRegions";
   private static final String PLANNER_INPUTS_FILENAME = "PlannerInputs.txt";

   public static List<DataSet> loadDataSets()
   {
      return loadDataSets(dataSet -> true);
   }

   public static List<DataSet> loadDataSets(Predicate<DataSet> dataSetFilter)
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
         DataSet dataSet = loadDataSet(dataSetName);
         dataSets.add(dataSet);
      }

      dataSets.removeIf(dataSetFilter.negate());
      return dataSets;
   }

   public static DataSet loadDataSet(String dataSetName)
   {
      Class<DataSetLoader> loadingClass = DataSetLoader.class;

      String dataSetPlanarRegionsPath = DATA_SET_DIRECTORY_PATH + "/" + dataSetName + "/" + PLANAR_REGIONS_DIRECTORY;
      PlanarRegionsList planarRegionsList = PlanarRegionFileTools.importPlanarRegionData(loadingClass.getClassLoader(), dataSetPlanarRegionsPath);

      DataSet dataSet = new DataSet(dataSetName, planarRegionsList);
      InputStream plannerInputsStream = loadingClass.getClassLoader()
                                                    .getResourceAsStream(DATA_SET_DIRECTORY_PATH + "/" + dataSetName + "/" + PLANNER_INPUTS_FILENAME);
      if(plannerInputsStream != null)
      {
         try
         {
            PlannerInput plannerInput = loadPlannerInputs(plannerInputsStream);
            dataSet.setPlannerInput(plannerInput);
         }
         catch(IOException e)
         {
            System.err.println("Unable to read planner inputs for dataset: " + dataSetName);
            return null;
         }
      }

      return dataSet;
   }

   private static PlannerInput loadPlannerInputs(InputStream inputStream) throws IOException
   {
      BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream, UTF_8));

      String line;
      PlannerInput plannerInput = new PlannerInput();

      while((line = reader.readLine()) != null)
      {
         if(line.equals(""))
            continue;

         String[] lineSubstrings = line.split(" ");
         switch (lineSubstrings[0])
         {
         case "startPosition":
         {
            plannerInput.setStartPosition(Double.parseDouble(lineSubstrings[1]), Double.parseDouble(lineSubstrings[2]), Double.parseDouble(lineSubstrings[3]));
            break;
         }
         case "goalPosition":
         {
            plannerInput.setGoalPosition(Double.parseDouble(lineSubstrings[1]), Double.parseDouble(lineSubstrings[2]), Double.parseDouble(lineSubstrings[3]));
            break;
         }
         case "startYaw":
         {
            plannerInput.setStartYaw(Double.parseDouble(lineSubstrings[1]));
            break;
         }
         case "goalYaw":
         {
            plannerInput.setGoalYaw(Double.parseDouble(lineSubstrings[1]));
            break;
         }
         default:
         {
            if(lineSubstrings.length > 1)
            {
               for (int i = 1; i < lineSubstrings.length; i++)
               {
                  plannerInput.addAdditionalData(lineSubstrings[0], lineSubstrings[i]);
               }
            }
            break;
         }
         }
      }

      return plannerInput;
   }
}
