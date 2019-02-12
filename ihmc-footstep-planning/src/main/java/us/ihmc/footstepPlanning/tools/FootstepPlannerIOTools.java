package us.ihmc.footstepPlanning.tools;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.BufferedReader;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerIOTools extends VisibilityGraphsIOTools
{
   public static final String TEST_DATA_URL = "unitTestDataSets/test";
   public static final String IN_DEVELOPMENT_TEST_DATA_URL = "unitTestDataSets/inDevelopment";

   private static final String TYPE_FIELD_OPEN = "<Type,";
   private static final String TYPE_FIELD_CLOSE = ",Type>";

   private static final String TIMEOUT_FIELD_OPEN = "<Timeout,";
   private static final String TIMEOUT_FIELD_CLOSE = ",Timeout>";

   private static final String START_ORIENTATION_FIELD_OPEN = "<StartOrientation,";
   private static final String START_ORIENTATION_FIELD_CLOSE = ",StartOrientation>";

   private static final String GOAL_ORIENTATION_FIELD_OPEN = "<GoalOrientation,";
   private static final String GOAL_ORIENTATION_FIELD_CLOSE = ",GoalOrientation>";

   public static class FootstepPlannerUnitTestDataset extends VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset
   {
      private List<FootstepPlannerType> plannerTypes;
      private Double[] timeouts;
      private Quaternion startOrientation;
      private Quaternion goalOrientation;

      private FootstepPlannerUnitTestDataset(Class<?> clazz, String datasetResourceName)
      {
         super(clazz, datasetResourceName);

         if (plannerTypes == null)
            throw new RuntimeException("Could not load the planner types. Data folder: " + datasetResourceName);
         if (timeouts == null)
            throw new RuntimeException("Could not load the timeouts. Data folder:" + datasetResourceName);
      }

      public List<FootstepPlannerType> getTypes()
      {
         return plannerTypes;
      }

      public double getTimeout(FootstepPlannerType plannerType)
      {
         if (plannerTypes.contains(plannerType))
            return timeouts[plannerTypes.indexOf(plannerType)];

         return Double.NaN;
      }

      public boolean hasStartOrientation()
      {
         return startOrientation != null;
      }

      public Quaternion getStartOrientation()
      {
         return startOrientation;
      }

      public Quaternion getGoalOrientation()
      {
         return goalOrientation;
      }

      public boolean hasGoalOrientation()
      {
         return goalOrientation != null;
      }

      @Override
      protected void loadFields(BufferedReader bufferedReader)
      {
         super.loadFields(bufferedReader);

         plannerTypes = parseField(bufferedReader, TYPE_FIELD_OPEN, TYPE_FIELD_CLOSE, FootstepPlannerIOTools::parsePlannerTypes);
         timeouts = parseField(bufferedReader, TIMEOUT_FIELD_OPEN, TIMEOUT_FIELD_CLOSE, FootstepPlannerIOTools::parsePlannerTimeouts);
         startOrientation = parseField(bufferedReader, START_ORIENTATION_FIELD_OPEN, START_ORIENTATION_FIELD_CLOSE, FootstepPlannerIOTools::parseQuaternion);
         goalOrientation = parseField(bufferedReader, GOAL_ORIENTATION_FIELD_OPEN, GOAL_ORIENTATION_FIELD_CLOSE, FootstepPlannerIOTools::parseQuaternion);
      }
   }

   public static boolean exportDataset(Path containingFolder, String datasetName, PlanarRegionsList planarRegionsList, Point3DReadOnly startPosition,
                                       QuaternionReadOnly startOrientation, Point3DReadOnly goalPosition, QuaternionReadOnly goalOrientation, FootstepPlannerType type, double timeout)
   {
      File datasetFolder = new File(containingFolder + File.separator + datasetName);
      if (datasetFolder.exists())
         return false;
      boolean success = datasetFolder.mkdir();
      if (!success)
         return false;

      Path planarRegionsFolder = Paths.get(datasetFolder.getPath() + File.separator + PlanarRegionFileTools.createDefaultTimeStampedFolderName());
      success = PlanarRegionFileTools.exportPlanarRegionData(planarRegionsFolder, planarRegionsList);
      if (!success)
         return false;

      success = exportParameters(datasetFolder, startPosition, startOrientation, goalPosition, goalOrientation, type, timeout);
      if (!success)
         return false;

      return true;
   }

   private static boolean exportParameters(File containingFolder, Point3DReadOnly startPosition, QuaternionReadOnly startOrientation,
                                           Point3DReadOnly goalPosition, QuaternionReadOnly goalOrientation, FootstepPlannerType type, double timeout)
   {

      if (containingFolder == null || !containingFolder.exists())
      {
         PrintTools.error("The given folder does not exist or is null.");
         return false;
      }

      if (startPosition == null || goalPosition == null || type == null || !Double.isFinite(timeout) || timeout < 0 )
      {
         PrintTools.error("Must export start, goal, planner type, AND timeout.");
         return false;
      }

      File parametersFile = new File(containingFolder.getAbsolutePath() + File.separator + INPUTS_PARAMETERS_FILENAME);
      writeField(parametersFile, START_FIELD_OPEN, START_FIELD_CLOSE, () -> getPoint3DString(startPosition));
      writeField(parametersFile, GOAL_FIELD_OPEN, GOAL_FIELD_END, () -> getPoint3DString(goalPosition));
      writeField(parametersFile, TYPE_FIELD_OPEN, TYPE_FIELD_CLOSE, () -> type.name());
      writeField(parametersFile, TIMEOUT_FIELD_OPEN, TIMEOUT_FIELD_CLOSE, () -> "" + timeout);

      if (startOrientation != null)
         writeField(parametersFile, START_ORIENTATION_FIELD_OPEN, START_ORIENTATION_FIELD_CLOSE, () -> getQuaternionString(startOrientation));
      if (goalOrientation != null)
         writeField(parametersFile, GOAL_ORIENTATION_FIELD_OPEN, GOAL_ORIENTATION_FIELD_CLOSE, () -> getQuaternionString(goalOrientation));

      return true;
   }

   public static List<FootstepPlannerUnitTestDataset> loadAllFootstepPlannerDatasetsWithoutOcclusions(Class<?> loadingClass)
   {
      return loadAllFootstepPlannerDatasets(loadingClass, TEST_DATA_URL);
   }

   public static List<FootstepPlannerUnitTestDataset> loadAllFootstepPlannerDatasetsWithoutOcclusionsInDevelopment(Class<?> loadingClass)
   {
      return loadAllFootstepPlannerDatasets(loadingClass, IN_DEVELOPMENT_TEST_DATA_URL);
   }

   public static List<FootstepPlannerUnitTestDataset> loadAllFootstepPlannerDatasets(Class<?> loadingClass, String dataURL)
   {
      List<String> childDirectories = PlanarRegionFileTools.listResourceDirectoryContents(loadingClass, dataURL);
      List<FootstepPlannerUnitTestDataset> datasets = new ArrayList<>();

      if (DEBUG && childDirectories.size() < 1)
         throw new RuntimeException("Unable to find the directory contents.");


      for (int i = 0; i < childDirectories.size(); i++)
      {
         PrintTools.info("trying to load:");
         PrintTools.info(dataURL + "/" + childDirectories.get(i));

         datasets.add(loadDataset(loadingClass, dataURL + "/" + childDirectories.get(i)));
      }

      if (DEBUG && datasets.size() < 1)
         throw new RuntimeException("Could not find any datasets.");

      return datasets;
   }

   public static FootstepPlannerUnitTestDataset loadDataset(Class<?> clazz, String datasetResourceName)
   {
      return new FootstepPlannerUnitTestDataset(clazz, datasetResourceName);
   }

   private static List<FootstepPlannerType> parsePlannerTypes(String stringPlannerTypes)
   {
      List<FootstepPlannerType> footstepPlannerTypes = new ArrayList<>();

      boolean containsData = true;

      while (containsData)
      {
         stringPlannerTypes = parsePlannerType(footstepPlannerTypes, stringPlannerTypes);

         if (stringPlannerTypes == null)
            containsData = false;
      }

      return footstepPlannerTypes;
   }

   private static String parsePlannerType(List<FootstepPlannerType> footstepPlannerTypes, String stringPlannerTypes)
   {
      if (stringPlannerTypes.contains(","))
      {
         footstepPlannerTypes.add(FootstepPlannerType.fromString(stringPlannerTypes.substring(0, stringPlannerTypes.indexOf(","))));
         stringPlannerTypes = stringPlannerTypes.substring(stringPlannerTypes.indexOf(",") + 1);

         while (stringPlannerTypes.startsWith(" "))
            stringPlannerTypes = stringPlannerTypes.replaceFirst(" ", "");

         return stringPlannerTypes;
      }

      footstepPlannerTypes.add(FootstepPlannerType.fromString(stringPlannerTypes));

      return null;
   }

   private static Double[] parsePlannerTimeouts(String stringTimeouts)
   {
      List<Double> footstepPlannerTimeouts = new ArrayList<>();

      boolean containsData = true;

      while (containsData)
      {
         stringTimeouts = parsePlannerTimeout(footstepPlannerTimeouts, stringTimeouts);

         if (stringTimeouts == null)
            containsData = false;
      }

      Double[] typeArray = new Double[footstepPlannerTimeouts.size()];
      footstepPlannerTimeouts.toArray(typeArray);

      return typeArray;
   }

   private static String parsePlannerTimeout(List<Double> footstepPlannerTimeouts, String stringTimeouts)
   {
      if (stringTimeouts.contains(","))
      {
         footstepPlannerTimeouts.add(Double.parseDouble(stringTimeouts.substring(0, stringTimeouts.indexOf(","))));
         stringTimeouts = stringTimeouts.substring(stringTimeouts.indexOf(",") + 1);

         while (stringTimeouts.startsWith(" "))
            stringTimeouts = stringTimeouts.replaceFirst(" ", "");

         return stringTimeouts;
      }

      footstepPlannerTimeouts.add(Double.parseDouble(stringTimeouts));

      return null;
   }

   private static Quaternion parseQuaternion(String stringQuaternion)
   {
      if (stringQuaternion.length() == 0)
         return null;

      double x = Double.parseDouble(stringQuaternion.substring(0, stringQuaternion.indexOf(",")));
      stringQuaternion = stringQuaternion.substring(stringQuaternion.indexOf(",") + 1);

      if (stringQuaternion.length() == 0)
         return null;

      double y = Double.parseDouble(stringQuaternion.substring(0, stringQuaternion.indexOf(",")));
      stringQuaternion = stringQuaternion.substring(stringQuaternion.indexOf(",") + 1);

      if (stringQuaternion.length() == 0)
         return null;

      double z = Double.parseDouble(stringQuaternion.substring(0, stringQuaternion.indexOf(",")));
      stringQuaternion = stringQuaternion.substring(stringQuaternion.indexOf(",") + 1);

      if (stringQuaternion.length() == 0)
         return null;

      double s = Double.parseDouble(stringQuaternion.substring(0));

      return new Quaternion(x, y, z, s);
   }

   protected static String getQuaternionString(QuaternionReadOnly quaternion)
   {
      return EuclidCoreIOTools.getStringOf("", "", ",", quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
   }

}
