package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public AtlasSwingPlannerParameters()
   {
      this("ihmc-open-robotics-software", "atlas/src/main/resources");
   }

   public AtlasSwingPlannerParameters(String projectName, String pathToResources)
   {
      super(SwingPlannerParameterKeys.keys, AtlasSwingPlannerParameters.class, projectName, pathToResources);
      load();
   }

   public static void main(String[] args)
   {
      AtlasSwingPlannerParameters parameters = new AtlasSwingPlannerParameters();
      parameters.save();
   }

}
