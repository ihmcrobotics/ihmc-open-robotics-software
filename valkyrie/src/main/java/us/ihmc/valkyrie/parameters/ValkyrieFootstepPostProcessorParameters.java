package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class ValkyrieFootstepPostProcessorParameters extends StoredPropertySet implements FootstepPostProcessingParametersBasics
{
   public ValkyrieFootstepPostProcessorParameters()
   {
      this("ihmc-open-robotics-software", "valkyrie/src/main/resources");
   }

   public ValkyrieFootstepPostProcessorParameters(String projectName, String pathToResources)
   {
      super(FootstepPlannerParameterKeys.keys, ValkyrieFootstepPostProcessorParameters.class, projectName, pathToResources);

      setSplitFractionProcessingEnabled(false);
      setSwingOverRegionsEnabled(false);

      load();
   }

   public static void main(String[] args)
   {
      ValkyrieFootstepPostProcessorParameters parameters = new ValkyrieFootstepPostProcessorParameters();
      parameters.save();
   }
}
