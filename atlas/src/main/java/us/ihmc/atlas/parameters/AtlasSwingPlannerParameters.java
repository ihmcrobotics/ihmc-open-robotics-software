package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public AtlasSwingPlannerParameters()
   {
      this("");
   }

   public AtlasSwingPlannerParameters(String fileNameSuffix)
   {
      this("ihmc-open-robotics-software", "atlas/src/main/resources", fileNameSuffix);
   }

   public AtlasSwingPlannerParameters(String projectName, String pathToResources)
   {
      this(projectName, pathToResources, "");
   }

   public AtlasSwingPlannerParameters(String projectName, String pathToResources, String fileNameSuffix)
   {
      super(SwingPlannerParameterKeys.keys, AtlasSwingPlannerParameters.class, projectName, pathToResources, fileNameSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      AtlasSwingPlannerParameters parameters = new AtlasSwingPlannerParameters();
      parameters.save();
   }
}
