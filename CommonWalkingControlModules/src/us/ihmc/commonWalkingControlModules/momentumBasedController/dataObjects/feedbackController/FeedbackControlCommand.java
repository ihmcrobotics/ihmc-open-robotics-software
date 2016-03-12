package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController;

public interface FeedbackControlCommand<T extends FeedbackControlCommand<T>>
{
   public enum FeedbackControlCommandType
   {
      JOINTSPACE_CONTROL,
      SPATIAL_CONTROL, ORIENTATION_CONTROL, POINT_CONTROL,
      COMMAND_LIST
   }

   public abstract void set(T other);

   public abstract FeedbackControlCommandType getCommandType();
}
