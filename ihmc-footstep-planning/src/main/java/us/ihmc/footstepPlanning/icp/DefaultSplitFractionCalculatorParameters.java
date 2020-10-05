package us.ihmc.footstepPlanning.icp;

import us.ihmc.tools.property.StoredPropertySet;

public class DefaultSplitFractionCalculatorParameters extends StoredPropertySet implements SplitFractionCalculatorParametersBasics
{
   public DefaultSplitFractionCalculatorParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   public DefaultSplitFractionCalculatorParameters(String projectName, String pathToResources) // for robots and UIs that want their own defaults and saves
   {
      this(projectName, pathToResources, null);
   }

   public DefaultSplitFractionCalculatorParameters(SplitFractionCalculatorParametersReadOnly footstepPostProcessingParameters) // for message passing or temp access
   {
      this("ihmc-open-robotics-software", "ihmc-footstep-planning/src/main/resources", footstepPostProcessingParameters);
   }

   private DefaultSplitFractionCalculatorParameters(String projectName, String pathToResources, SplitFractionCalculatorParametersReadOnly footstepPostProcessingParameters)
   {
      super(SplitFractionCalculatorParameterKeys.keys, DefaultSplitFractionCalculatorParameters.class, projectName, pathToResources);

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
      DefaultSplitFractionCalculatorParameters parameters = new DefaultSplitFractionCalculatorParameters();
      parameters.save();
   }
}
