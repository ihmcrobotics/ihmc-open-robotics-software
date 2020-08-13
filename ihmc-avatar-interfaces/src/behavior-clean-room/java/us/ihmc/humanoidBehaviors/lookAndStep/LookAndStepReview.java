package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.thread.ResettableSingleThreadExecutor;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;

import java.util.function.Consumer;

public class LookAndStepReview<T>
{
   private volatile boolean beingReviewed = false;

   private StatusLogger statusLogger;
   private String description;
   private TypedNotification<Boolean> approvalNotification;
   private Consumer<T> callback;
   private ResettableSingleThreadExecutor executor;

   private T data;

   public void initialize(StatusLogger statusLogger, String description, TypedNotification<Boolean> approvalNotification, Consumer<T> callback)
   {
      this.statusLogger = statusLogger;
      this.description = description;
      this.approvalNotification = approvalNotification;
      this.callback = callback;

      executor = new ResettableSingleThreadExecutor(FormattingTools.titleToPascalCase(description) + "Review");
   }

   public void review(T data)
   {
      this.data = data;

      executor.queueExecution(this::performReviewTask);
   }

   private void performReviewTask()
   {
      beingReviewed = true;
      approvalNotification.poll(); // to make sure to wait for a new review notification
      statusLogger.info("Waiting for {} operator review...", description);
      try
      {
         boolean approved = approvalNotification.blockingPoll(DefaultExceptionHandler.PROCEED_SILENTLY);
         statusLogger.info("Operator reviewed {}: {}", description, approved);
         beingReviewed = false;

         if (approved)
         {
            callback.accept(data);
         }
      }
      catch (NullPointerException nullPointerException)
      {
         // An NPE happens here when reset is called. I still can't find why but it doesn't really matter so do nothing
      }
   }

   public boolean isBeingReviewed()
   {
      return beingReviewed;
   }

   public void reset()
   {
      executor.interruptAndReset();
      beingReviewed = false;
   }
}
