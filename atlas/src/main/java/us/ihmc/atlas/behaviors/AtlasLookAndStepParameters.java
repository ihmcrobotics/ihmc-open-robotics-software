package us.ihmc.atlas.behaviors;

import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.log.LogTools;

/**
 * The JSON file for this property set is located here:
 * atlas/src/main/resources/us/ihmc/atlas/behaviors/LookAndStepParametersForAtlas.json
 */
public class AtlasLookAndStepParameters extends LookAndStepBehaviorParameters
{
   public AtlasLookAndStepParameters()
   {
      super(AtlasLookAndStepParameters.class, "ForAtlas");
   }

   public static void main(String[] args)
   {
      AtlasLookAndStepParameters parameters = new AtlasLookAndStepParameters();
      LogTools.info(parameters.getTitle());
   }
}
