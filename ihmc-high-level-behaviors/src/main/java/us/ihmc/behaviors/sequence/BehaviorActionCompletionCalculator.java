package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.tools.Timer;

/**
 * This class will probably change a lot as actions have
 * more complex conditions and decision-making logic. It'll
 * morph into something else entirely, probably.
 */
public class BehaviorActionCompletionCalculator
{
   private double translationError;
   private double rotationError;

   public enum Component
   {
      TRANSLATION, ORIENTATION
   }

   public boolean isComplete(FramePose3DReadOnly desired,
                             FramePose3DReadOnly actual,
                             double translationTolerance,
                             double rotationTolerance,
                             double actionNominalDuration,
                             Timer executionTimer,
                             Component... components)
   {
      boolean timeIsUp = !executionTimer.isRunning(actionNominalDuration);
      boolean desiredPoseAchieved = timeIsUp;
      for (Component component : components)
      {
         switch (component)
         {
            case TRANSLATION ->
            {
               translationError = actual.getTranslation().differenceNorm(desired.getTranslation());
               desiredPoseAchieved = desiredPoseAchieved && (translationError <= translationTolerance);
            }
            case ORIENTATION ->
            {
               rotationError = actual.getRotation().distance(desired.getRotation(), true);
               desiredPoseAchieved = desiredPoseAchieved && (rotationError <= rotationTolerance);
            }
            default -> throw new IllegalStateException("Unexpected value: " + component);
         }
      }
      return desiredPoseAchieved;
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
