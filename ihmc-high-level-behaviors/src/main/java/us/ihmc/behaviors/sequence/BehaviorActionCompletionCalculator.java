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
   private double prevTranslationError = Double.POSITIVE_INFINITY;
   private double prevRotationError = Double.POSITIVE_INFINITY;
   public static final double MIN_POSITION_CHANGE = 0.001;
   public static final double MIN_ORIENTATION_CHANGE = Math.toRadians(1.0);

   public boolean isComplete(FramePose3DReadOnly desired,
                             FramePose3DReadOnly actual,
                             double translationTolerance,
                             double rotationTolerance,
                             double actionNominalDuration,
                             Timer executionTimer,
                             BehaviorActionCompletionComponent... components)
   {
      boolean timeIsUp = !executionTimer.isRunning(actionNominalDuration);
      boolean desiredPoseAchieved = timeIsUp;
      for (BehaviorActionCompletionComponent component : components)
      {
         switch (component)
         {
            case TRANSLATION ->
            {
               translationError = actual.getTranslation().differenceNorm(desired.getTranslation());
               desiredPoseAchieved &= ((translationError <= translationTolerance) && !(translationError < prevTranslationError - MIN_POSITION_CHANGE));
               prevTranslationError = translationError;
            }
            case ORIENTATION ->
            {
               // TODO: compare current error with previous error to wait before declaring that the action is finished
               rotationError = actual.getRotation().distance(desired.getRotation(), true);
               desiredPoseAchieved &= ((rotationError <= rotationTolerance) && !(rotationError < prevRotationError - MIN_ORIENTATION_CHANGE));
               prevRotationError = rotationError;
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
