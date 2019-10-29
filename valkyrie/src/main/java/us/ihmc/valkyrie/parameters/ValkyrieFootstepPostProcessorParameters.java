package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingKeys;
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
      super(FootstepPostProcessingKeys.keys, ValkyrieFootstepPostProcessorParameters.class, projectName, pathToResources);

      setPositionSplitFractionProcessingEnabled(false);
      setSwingOverRegionsProcessingEnabled(false);

      load();
   }

   public static void main(String[] args)
   {
      ValkyrieFootstepPostProcessorParameters parameters = new ValkyrieFootstepPostProcessorParameters();
      parameters.save();
   }
}
