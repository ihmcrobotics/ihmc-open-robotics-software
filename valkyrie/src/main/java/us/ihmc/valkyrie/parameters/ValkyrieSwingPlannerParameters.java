package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class ValkyrieSwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public ValkyrieSwingPlannerParameters()
   {
      this("");
   }

   public ValkyrieSwingPlannerParameters(String fileNameSuffix)
   {
      this("ihmc-open-robotics-software", "valkyrie/src/main/resources", fileNameSuffix);
   }

   public ValkyrieSwingPlannerParameters(String projectName, String pathToResources)
   {
      this(projectName, pathToResources, "");
   }

   public ValkyrieSwingPlannerParameters(String projectName, String pathToResources, String fileNameSuffix)
   {
      super(SwingPlannerParameterKeys.keys, ValkyrieSwingPlannerParameters.class, projectName, pathToResources, fileNameSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      ValkyrieSwingPlannerParameters parameters = new ValkyrieSwingPlannerParameters();
      parameters.save();
   }
}
