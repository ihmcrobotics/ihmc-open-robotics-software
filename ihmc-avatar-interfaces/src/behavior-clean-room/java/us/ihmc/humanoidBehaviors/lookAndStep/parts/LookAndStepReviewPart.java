package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;

import java.util.function.Consumer;

public class LookAndStepReviewPart<T>
{
   private volatile boolean beingReviewed = false;

   private final String description;
   private final TypedNotification<Boolean> approvalNotification;
   private final Consumer<T> callback;

   public LookAndStepReviewPart(String description, TypedNotification<Boolean> approvalNotification, Consumer<T> callback)
   {
      this.description = description;
      this.approvalNotification = approvalNotification;
      this.callback = callback;
   }

   public void review(T data)
   {
      ThreadTools.startAsDaemon(() ->
      {
         beingReviewed = true;
         LogTools.info("Waiting for {} operator review...", description);
         boolean approved = approvalNotification.blockingPoll();
         LogTools.info("Operator reviewed {}: {}", description, approved);
         beingReviewed = false;

         if (approved)
         {
            callback.accept(data);
         }
      }, description + "Review");
   }

   public boolean isBeingReviewed()
   {
      return beingReviewed;
   }
}
