package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController;

public abstract class FeedbackControlCommand<T extends FeedbackControlCommand<T>>
{
   public enum FeedbackControlCommandType
   {
      JOINTSPACE_CONTROL,
      SPATIAL_CONTROL, ORIENTATION_CONTROL, POINT_CONTROL,
      COMMAND_LIST
   }

   private final FeedbackControlCommandType commandType;

   public FeedbackControlCommand(FeedbackControlCommandType commandType)
   {
      this.commandType = commandType;
   }

   public abstract void set(T other);

   public FeedbackControlCommandType getCommandType()
   {
      return commandType;
   }
}
