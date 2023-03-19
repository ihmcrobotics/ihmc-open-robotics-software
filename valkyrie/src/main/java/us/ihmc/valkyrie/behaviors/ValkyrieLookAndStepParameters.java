package us.ihmc.valkyrie.behaviors;

import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.log.LogTools;

/**
 * The JSON file for this property set is located here:
 * valkyrie/src/main/resources/us/ihmc/valkyrie/behaviors/LookAndStepParametersForValkyrie.json
 */
public class ValkyrieLookAndStepParameters extends LookAndStepBehaviorParameters
{
   public ValkyrieLookAndStepParameters()
   {
      super(ValkyrieLookAndStepParameters.class, "ForValkyrie");
   }

   public static void main(String[] args)
   {
      ValkyrieLookAndStepParameters parameters = new ValkyrieLookAndStepParameters();
      LogTools.info(parameters.getTitle());
   }
}
