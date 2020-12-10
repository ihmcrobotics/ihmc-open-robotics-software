package us.ihmc.atlas.parameters;

import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasVisibilityGraphParameters extends StoredPropertySet implements VisibilityGraphsParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String PATH_TO_RESOURCES = "atlas/src/main/resources";

   public AtlasVisibilityGraphParameters()
   {
      this("");
   }

   public AtlasVisibilityGraphParameters(String fileNameSuffix)
   {
      this(PROJECT_NAME, PATH_TO_RESOURCES, fileNameSuffix);
   }

   public AtlasVisibilityGraphParameters(String projectName, String pathToResources)
   {
      this(projectName, pathToResources, "");
   }

   public AtlasVisibilityGraphParameters(String projectName, String pathToResources, String fileNameSuffix)
   {
      super(VisibilityGraphParametersKeys.keys, AtlasVisibilityGraphParameters.class, projectName, pathToResources, fileNameSuffix);
      load();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(VisibilityGraphParametersKeys.keys,
                                                                  AtlasVisibilityGraphParameters.class,
                                                                  PROJECT_NAME,
                                                                  PATH_TO_RESOURCES);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
