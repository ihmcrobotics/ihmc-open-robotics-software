package us.ihmc.footstepPlanning.tools;

import us.ihmc.commons.PrintTools;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.robotics.PlanarRegionFileTools;

import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerIOTools extends VisibilityGraphsIOTools
{

   private static final String TYPE_FIELD_OPEN = "<Type,";
   private static final String TYPE_FIELD_CLOSE = ",Type>";

   public static class FootstepPlannerUnitTestDataset extends VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset
   {
      private FootstepPlannerType[] plannerTypes;

      private FootstepPlannerUnitTestDataset(Class<?> clazz, String datasetResourceName)
      {
         super(clazz, datasetResourceName);

         if (plannerTypes == null)
            throw new RuntimeException("Could not load the planner types. Data folder: " + datasetResourceName);
      }

      @Override
      protected void loadFields(BufferedReader bufferedReader)
      {
         super.loadFields(bufferedReader);

         plannerTypes = parseField(bufferedReader, TYPE_FIELD_OPEN, TYPE_FIELD_CLOSE, FootstepPlannerIOTools::parsePlannerTypes);
      }
   }

   public static List<FootstepPlannerUnitTestDataset> loadAllFootstepPlannerDatasets(Class<?> loadingClass)
   {
      List<String> childDirectories = PlanarRegionFileTools.listResourceDirectoryContents(loadingClass, TEST_DATA_URL);
      List<FootstepPlannerUnitTestDataset> datasets = new ArrayList<>();

      for (int i = 0; i < childDirectories.size(); i++)
      {
         PrintTools.info("trying to load:");
         PrintTools.info(TEST_DATA_URL + "/" + childDirectories.get(i));

         datasets.add(loadDataset(loadingClass, TEST_DATA_URL + "/" + childDirectories.get(i)));
      }

      return datasets;
   }

   public static FootstepPlannerUnitTestDataset loadDataset(Class<?> clazz, String datasetResourceName)
   {
      return new FootstepPlannerUnitTestDataset(clazz, datasetResourceName);
   }


   private static FootstepPlannerType[] parsePlannerTypes(String stringPlannerTypes)
   {
      List<FootstepPlannerType> footstepPlannerTypes = new ArrayList<>();

      boolean containsData = true;

      while (containsData)
      {
         stringPlannerTypes = parsePlannerType(footstepPlannerTypes, stringPlannerTypes);

         if (stringPlannerTypes == null)
            containsData = false;
      }

      FootstepPlannerType[] typeArray = new FootstepPlannerType[footstepPlannerTypes.size()];
      footstepPlannerTypes.toArray(typeArray);

      return typeArray;
   }

   private static String parsePlannerType(List<FootstepPlannerType> footstepPlannerTypes, String stringPlannerTypes)
   {
      if (stringPlannerTypes.contains(","))
      {
         footstepPlannerTypes.add(FootstepPlannerType.fromString(stringPlannerTypes.substring(0, stringPlannerTypes.indexOf(","))));
         stringPlannerTypes = stringPlannerTypes.substring(stringPlannerTypes.indexOf(",") + 1);

         return stringPlannerTypes;
      }

      return null;
   }
}
