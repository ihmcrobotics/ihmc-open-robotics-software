package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.Throttler;

public class BehaviorActionCompletionCalculator
{
   private double translationError;
   private double rotationError;
   private boolean desiredTranslationAcheived;
   private boolean desiredRotationAcheived;
   private boolean desiredPoseAchieved;

   public boolean isComplete(FramePose3DReadOnly desired,
                             FramePose3DReadOnly actual,
                             double translationTolerance,
                             double rotationTolerance,
                             double actionNominalDuration,
                             Timer executionTimer,
                             Throttler warningThrottler)
   {
      boolean timeIsUp = !executionTimer.isRunning(actionNominalDuration);

      translationError = actual.getTranslation().differenceNorm(desired.getTranslation());
      desiredTranslationAcheived = translationError <= translationTolerance;

      rotationError = actual.getRotation().distance(desired.getRotation(), true);
      desiredRotationAcheived = rotationError <= rotationTolerance;

      desiredPoseAchieved = desiredTranslationAcheived && desiredRotationAcheived;

      return timeIsUp && desiredPoseAchieved;
   }

   public double getTranslationError()
   {
      return translationError;
   }

   public double getRotationError()
   {
      return rotationError;
   }
}
