package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public interface FeedbackControlCommand<T extends FeedbackControlCommand<T>>
{
   public abstract void set(T other);

   public abstract ControllerCoreCommandType getCommandType();
}
