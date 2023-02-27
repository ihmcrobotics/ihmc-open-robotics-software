package us.ihmc.valkyrie.behaviors;

import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.log.LogTools;

/**
 * The JSON file for this property set is located here:
 * valkyrie/src/main/resources/us/ihmc/valkyrie/behaviors/LookAndStepParametersForValkyrie.json
 */
public class ValkyrieLookAndStepParameters extends LookAndStepBehaviorParameters
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "valkyrie/src/main/resources";

   public ValkyrieLookAndStepParameters()
   {
      super(ValkyrieLookAndStepParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, "ForValkyrie");
   }

   public static void main(String[] args)
   {
      ValkyrieLookAndStepParameters parameters = new ValkyrieLookAndStepParameters();
      LogTools.info(parameters.getTitle());
   }
}
