package us.ihmc.pathPlanning;

import org.apache.commons.io.Charsets;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Predicate;

import static java.nio.charset.StandardCharsets.UTF_8;

public class DataSetIOTools
{
   public static final String RESOURCES_DIRECTORY = "ihmc-open-robotics-software/ihmc-path-planning/src/data-sets/resources";
   public static final String DATA_SET_DIRECTORY_PATH = "us/ihmc/pathPlanning/dataSets";

   private static final String DATA_SET_LIST_FILENAME = "DataSetList.txt";
   private static final String PLANAR_REGIONS_DIRECTORY = "PlanarRegions";
   private static final String PLANNER_INPUTS_FILENAME = "PlannerInputs.txt";

   public static List<DataSet> loadDataSets()
   {
      return loadDataSets(dataSet -> true);
   }

   public static List<DataSet> loadDataSets(Predicate<DataSet> dataSetFilter)
   {
      List<String> dataSetNamesList;

      try
      {
         dataSetNamesList = loadDataSetNames();
      }
      catch (IOException e)
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

   private static List<String> loadDataSetNames() throws IOException
   {
      Class<?> loadingClass = DataSetIOTools.class;
      InputStream dataSetListStream = loadingClass.getResourceAsStream(DATA_SET_LIST_FILENAME);
      InputStreamReader reader = new InputStreamReader(dataSetListStream, Charsets.toCharset(UTF_8));
      BufferedReader bufferedReader = new BufferedReader(reader);

      List<String> dataSetNamesList = new ArrayList<>();
      String line = bufferedReader.readLine();
      while (line != null) {
         dataSetNamesList.add(line);
         line = bufferedReader.readLine();
      }

      return dataSetNamesList;

   }

   public static DataSet loadDataSet(String dataSetName)
   {
      Class<DataSetIOTools> loadingClass = DataSetIOTools.class;

      String dataSetPlanarRegionsPath = DATA_SET_DIRECTORY_PATH + "/" + dataSetName + "/" + PLANAR_REGIONS_DIRECTORY;
      PlanarRegionsList planarRegionsList = PlanarRegionFileTools.importPlanarRegionData(loadingClass.getClassLoader(), dataSetPlanarRegionsPath);

      DataSet dataSet = new DataSet(dataSetName, planarRegionsList);
      InputStream plannerInputsStream = loadingClass.getClassLoader()
                                                    .getResourceAsStream(DATA_SET_DIRECTORY_PATH + "/" + dataSetName + "/" + PLANNER_INPUTS_FILENAME);
      if (plannerInputsStream != null)
      {
         try
         {
            PlannerInput plannerInput = loadPlannerInputs(plannerInputsStream);
            dataSet.setPlannerInput(plannerInput);
         }
         catch (IOException e)
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

      while ((line = reader.readLine()) != null)
      {
         if (line.equals(""))
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
            if (lineSubstrings.length > 1)
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

   public static boolean exportDataSet(DataSet dataSet)
   {
      String exportDirectory = RESOURCES_DIRECTORY.replace('/', File.separatorChar) + File.separator + DATA_SET_DIRECTORY_PATH.replace('/', File.separatorChar);
      return exportDataSet(exportDirectory, dataSet);
   }

   public static boolean exportDataSet(String exportDirectory, DataSet dataSet)
   {
      try
      {
         String dataSetPath = exportDirectory + File.separator + dataSet.getName();
         File dataSetFile = new File(dataSetPath);
         if (dataSetFile.exists())
         {
            System.err.println("Unable to export dataset, file already exists");
            return false;
         }

         Files.createDirectories(new File(dataSetPath).toPath());

         Path planarRegionsPath = new File(dataSetPath + File.separator + PLANAR_REGIONS_DIRECTORY).toPath();
         Files.createDirectories(planarRegionsPath);
         PlanarRegionFileTools.exportPlanarRegionData(planarRegionsPath, dataSet.getPlanarRegionsList());

         if(!dataSet.hasPlannerInput())
            return true;

         File plannerInputsFile = new File(dataSetPath + File.separator + PLANNER_INPUTS_FILENAME);
         plannerInputsFile.createNewFile();
         exportPlannerInputs(plannerInputsFile, dataSet.getPlannerInput());

         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   private static void exportPlannerInputs(File file, PlannerInput plannerInput) throws IOException
   {
      FileWriter fileWriter = new FileWriter(file);

      fileWriter.write("startPosition " + plannerInput.getStartPosition().getX() + " " + plannerInput.getStartPosition().getY() + " " + plannerInput.getStartPosition().getZ());
      fileWriter.write("\n");

      fileWriter.write("goalPosition " + plannerInput.getGoalPosition().getX() + " " + plannerInput.getGoalPosition().getY() + " " + plannerInput.getGoalPosition().getZ());
      fileWriter.write("\n");

      fileWriter.write("startYaw " + plannerInput.getStartYaw());
      fileWriter.write("\n");

      fileWriter.write("goalYaw " + plannerInput.getGoalYaw());
      fileWriter.write("\n");

      HashMap<String, List<String>> additionalDataMap = plannerInput.getAdditionDataMap();
      for(String key : additionalDataMap.keySet())
      {
         StringBuilder stringBuilder = new StringBuilder();
         stringBuilder.append(key + " ");

         for(String value : additionalDataMap.get(key))
         {
            stringBuilder.append(value + " ");
         }

         fileWriter.write(stringBuilder.toString());
         fileWriter.write("\n");
      }

      fileWriter.flush();
      fileWriter.close();
   }
}
