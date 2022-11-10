package us.ihmc.atlas.behaviors;

import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.log.LogTools;

/**
 * The JSON file for this property set is located here:
 * atlas/src/main/resources/us/ihmc/atlas/behaviors/LookAndStepParametersForAtlas.json
 */
public class AtlasLookAndStepParameters extends LookAndStepBehaviorParameters
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "atlas/src/main/resources";

   public AtlasLookAndStepParameters()
   {
      super(AtlasLookAndStepParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, "ForAtlas");
   }

   public static void main(String[] args)
   {
      AtlasLookAndStepParameters parameters = new AtlasLookAndStepParameters();
      LogTools.info(parameters.getTitle());
   }
}
