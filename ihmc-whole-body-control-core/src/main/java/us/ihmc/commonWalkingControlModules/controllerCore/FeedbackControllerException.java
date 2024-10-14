package us.ihmc.commonWalkingControlModules.controllerCore;

/**
 * Runtime exception to indicate that the {@link WholeBodyFeedbackController} or one of its inner
 * components has faced an illegal state.
 */
public class FeedbackControllerException extends RuntimeException
{
   private static final long serialVersionUID = -8354238369002780518L;

   /**
    * Constructs a {@code FeedbackControllerException} with no detail message.
    */
   public FeedbackControllerException()
   {
      super();
   }

   /**
    * Constructs a {@code FeedbackControllerException} with the specified detail message.
    *
    * @param message the detailed message.
    */
   public FeedbackControllerException(String message)
   {
      super(message);
   }

   /**
    * Constructs a {@code FeedbackControllerException} with the specified detail message and cause.
    *
    * @param message the detailed message.
    * @param cause   the cause.
    */
   public FeedbackControllerException(String message, Throwable cause)
   {
      super(message, cause);
   }

   /**
    * Constructs a {@code FeedbackControllerException} with the specified cause.
    *
    * @param cause the cause.
    */
   public FeedbackControllerException(Throwable cause)
   {
      super(cause);
   }
}
