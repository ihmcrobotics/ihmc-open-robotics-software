package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.tools.NonWallTimer;

/**
 * This class will probably change a lot as actions have
 * more complex conditions and decision-making logic. It'll
 * morph into something else entirely, probably.
 */
public class BehaviorActionCompletionCalculator
{
   private double translationError;
   private double rotationError;

   public boolean isComplete(FramePose3DReadOnly desired,
                             FramePose3DReadOnly actual,
                             double translationTolerance,
                             double rotationTolerance,
                             double actionNominalDuration,
                             NonWallTimer executionTimer,
                             ActionNodeState<?> state,
                             BehaviorActionCompletionComponent... components)
   {
      boolean timeIsUp = !executionTimer.isRunning(actionNominalDuration);
      // Don't allow it to go more than 50% longer
      boolean hitTimeLimit = !executionTimer.isRunning(actionNominalDuration * 1.5);
      if (hitTimeLimit)
         state.setFailed(true); // This is a failure which should abort automatic execution

      boolean desiredPoseAchieved = timeIsUp;
      for (BehaviorActionCompletionComponent component : components)
      {
         switch (component)
         {
            case TRANSLATION ->
            {
               translationError = actual.getTranslation().differenceNorm(desired.getTranslation());
               desiredPoseAchieved &= (translationError <= translationTolerance);
            }
            case ORIENTATION ->
            {
               rotationError = actual.getRotation().distance(desired.getRotation(), true);
               desiredPoseAchieved &= (rotationError <= rotationTolerance);
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
