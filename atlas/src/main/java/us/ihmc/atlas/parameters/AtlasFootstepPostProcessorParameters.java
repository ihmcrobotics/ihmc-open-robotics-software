package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingKeys;
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
      super(FootstepPostProcessingKeys.keys, AtlasFootstepPostProcessorParameters.class, projectName, pathToResources);

      setPositionSplitFractionProcessingEnabled(true);
      setTransferSplitFractionAtFullDepth(0.3);
      setTransferWeightDistributionAtFullDepth(0.75);
      setStepHeightForLargeStepDown(0.1);
      setLargestStepDownHeight(0.3);

      load();
   }

   public static void main(String[] args)
   {
      AtlasFootstepPostProcessorParameters parameters = new AtlasFootstepPostProcessorParameters();
      parameters.save();
   }
}