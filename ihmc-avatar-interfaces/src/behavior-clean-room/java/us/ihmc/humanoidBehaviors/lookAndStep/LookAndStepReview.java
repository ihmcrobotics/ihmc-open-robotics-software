package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
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

   public void initialize(StatusLogger statusLogger, String description, TypedNotification<Boolean> approvalNotification, Consumer<T> callback)
   {
      this.statusLogger = statusLogger;
      this.description = description;
      this.approvalNotification = approvalNotification;
      this.callback = callback;
   }

   public void review(T data)
   {
      ThreadTools.startAsDaemon(() ->
      {
         beingReviewed = true;
         statusLogger.info("Waiting for {} operator review...", description);
         boolean approved = false;
         try
         {
            approved = approvalNotification.blockingPoll();
         }
         catch (NullPointerException e)
         {
            statusLogger.error("Exception: {}, {}", approvalNotification, e.getMessage());
            e.printStackTrace();
            throw e;
         }
         statusLogger.info("Operator reviewed {}: {}", description, approved);
         beingReviewed = false;

         if (approved)
         {
            callback.accept(data);
         }
      }, FormattingTools.titleToPascalCase(description) + "Review");
   }

   public boolean isBeingReviewed()
   {
      return beingReviewed;
   }
}
