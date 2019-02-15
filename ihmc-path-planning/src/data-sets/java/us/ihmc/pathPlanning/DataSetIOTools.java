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

   public static final String START_POSITION_TAG = "start_position";
   public static final String GOAL_POSITION_TAG = "goal_position";
   public static final String START_YAW_TAG = "start_yaw";
   public static final String GOAL_YAW_TAG = "goal_yaw";
   public static final String VIS_GRAPH_TAG = "vis_graph_status";
   public static final String STEP_PLANNERS_TAG = "step_planner_status";
   public static final String TIMEOUT_SUFFIX = "_timeout";
   public static final String QUADRUPED_PLANNER_TAG = "quadruped_planner_status";
   public static final String QUADRUPED_TIMEOUT_TAG = "quadruped" + TIMEOUT_SUFFIX;

   public static final String TESTABLE_FLAG = "test";
   public static final String IN_DEVELOPMENT_FLAG = "dev";

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
         throw new RuntimeException("Unable to read list of data set directories");
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
      InputStream dataSetListStream = loadingClass.getResourceAsStream("/" + DATA_SET_DIRECTORY_PATH);

      InputStreamReader reader = new InputStreamReader(dataSetListStream, Charsets.toCharset(UTF_8));
      BufferedReader bufferedReader = new BufferedReader(reader);

      List<String> dataSetNamesList = new ArrayList<>();
      String line = bufferedReader.readLine();
      while (line != null)
      {
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
         case START_POSITION_TAG:
         {
            plannerInput.setStartPosition(Double.parseDouble(lineSubstrings[1]), Double.parseDouble(lineSubstrings[2]), Double.parseDouble(lineSubstrings[3]));
            break;
         }
         case GOAL_POSITION_TAG:
         {
            plannerInput.setGoalPosition(Double.parseDouble(lineSubstrings[1]), Double.parseDouble(lineSubstrings[2]), Double.parseDouble(lineSubstrings[3]));
            break;
         }
         case START_YAW_TAG:
         {
            plannerInput.setStartYaw(Double.parseDouble(lineSubstrings[1]));
            break;
         }
         case GOAL_YAW_TAG:
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

      fileWriter.write(START_POSITION_TAG + " " + plannerInput.getStartPosition().getX() + " " + plannerInput.getStartPosition().getY() + " " + plannerInput.getStartPosition().getZ());
      fileWriter.write("\n");

      fileWriter.write(GOAL_POSITION_TAG + " " + plannerInput.getGoalPosition().getX() + " " + plannerInput.getGoalPosition().getY() + " " + plannerInput.getGoalPosition().getZ());
      fileWriter.write("\n");

      fileWriter.write(START_YAW_TAG + " " + plannerInput.getStartYaw());
      fileWriter.write("\n");

      fileWriter.write(GOAL_YAW_TAG + " " + plannerInput.getGoalYaw());
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

   public static boolean isDataSetFile(File file, boolean requirePlannerInputsFile)
   {
      if(!file.exists() || !file.isDirectory())
         return false;

      File[] files = file.listFiles();
      boolean hasPlanarRegions = false;
      boolean hasPlannerInputs = false;

      for (int i = 0; i < files.length; i++)
      {
         if(files[i].isDirectory() && files[i].getName().equals(PLANAR_REGIONS_DIRECTORY))
            hasPlanarRegions = true;
         if(files[i].getName().equals(PLANNER_INPUTS_FILENAME))
            hasPlannerInputs = true;
      }

      return hasPlanarRegions && (hasPlannerInputs || !requirePlannerInputsFile);
   }
}
