package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String PATH_TO_RESOURCES = "atlas/src/main/resources";

   public AtlasFootstepPlannerParameters()
   {
      this("");
   }

   public AtlasFootstepPlannerParameters(String fileNameSuffix)
   {
      this(PROJECT_NAME, PATH_TO_RESOURCES, fileNameSuffix);
   }

   public AtlasFootstepPlannerParameters(String projectName, String pathToResources)
   {
      this(projectName, pathToResources, "");
   }

   public AtlasFootstepPlannerParameters(String projectName, String pathToResources, String fileNameSuffix)
   {
      super(FootstepPlannerParameterKeys.keys, AtlasFootstepPlannerParameters.class, projectName, pathToResources, fileNameSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(FootstepPlannerParameterKeys.keys,
                                                                  AtlasFootstepPlannerParameters.class,
                                                                  PROJECT_NAME,
                                                                  PATH_TO_RESOURCES);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}