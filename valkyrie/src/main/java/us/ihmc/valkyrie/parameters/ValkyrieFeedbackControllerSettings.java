package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;

public class ValkyrieFeedbackControllerSettings implements FeedbackControllerSettings
{
   @Override
   public boolean enableIntegralTerm()
   {
      return false; // Saves about 130 YoVariables.
   }
}
