package us.ihmc.commonWalkingControlModules.momentumControlCore;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;

public interface HeightController<T extends FeedbackControlCommand<T>>
{
   void compute(T command);

   double getHeightAcceleration();
}
