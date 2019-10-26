package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasFootstepPostProcessorParameters extends StoredPropertySet implements FootstepPostProcessingParametersBasics
{
   public AtlasFootstepPostProcessorParameters()
   {
      this("ihmc-open-robotics-software", "atlas/src/main/resources");
   }

   public AtlasFootstepPostProcessorParameters(String projectName, String pathToResources)
   {
      super(FootstepPlannerParameterKeys.keys, AtlasFootstepPostProcessorParameters.class, projectName, pathToResources);

      load();
   }

   public static void main(String[] args)
   {
      AtlasFootstepPostProcessorParameters parameters = new AtlasFootstepPostProcessorParameters();
      parameters.save();
   }
}