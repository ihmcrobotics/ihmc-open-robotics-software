package us.ihmc.footstepPlanning.postProcessing.parameters;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultFootstepPostProcessingParameters extends StoredPropertySet implements FootstepPostProcessingParametersBasics
{
   public DefaultFootstepPostProcessingParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   public DefaultFootstepPostProcessingParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultFootstepPostProcessingParameters(FootstepPostProcessingParametersReadOnly footstepPostProcessingParameters) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-footstep-planning/src/main/resources", footstepPostProcessingParameters);
   }

   private DefaultFootstepPostProcessingParameters(String projectName, String pathToResources, FootstepPostProcessingParametersReadOnly footstepPostProcessingParameters)
   {
      super(FootstepPostProcessingKeys.keys, DefaultFootstepPostProcessingParameters.class, projectName, pathToResources);

      if (footstepPostProcessingParameters != null)
      {
         set(footstepPostProcessingParameters);
      }
      else
      {
         load();
      }
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      DefaultFootstepPostProcessingParameters parameters = new DefaultFootstepPostProcessingParameters();
      parameters.save();
   }
}
