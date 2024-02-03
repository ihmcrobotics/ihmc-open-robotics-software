package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

/**
 * This class provides methods for evaluating the tracking error of
 * pose trajectories executed by the robot.
 */
public class TaskspaceTrajectoryTrackingErrorCalculator extends TrajectoryDurationTracker
{
   private double positionError;
   private double orientationError;
   private boolean isWithinPositionTolerance;

   public void reset()
   {
      super.reset();

      positionError = Double.POSITIVE_INFINITY;
      orientationError = Double.POSITIVE_INFINITY;
      isWithinPositionTolerance = false;
   }

   public void computeExecutionTimings(double nominalExecutionDuration)
   {
      computeExecutionTimings(nominalExecutionDuration, nominalExecutionDuration * 1.5);
   }

   public void computePoseTrackingData(FramePose3DReadOnly desired, FramePose3DReadOnly actual)
   {
      positionError = actual.getTranslation().differenceNorm(desired.getTranslation());
      orientationError = actual.getRotation().distance(desired.getRotation(), true);

      isWithinPositionTolerance = true;
   }

   /** Factors in errors in x,z,y translational Euclidean (R3) space. */
   public void factorInR3Errors(double positionErrorTolerance)
   {
      isWithinPositionTolerance &= positionError <= positionErrorTolerance;
   }

   /** Factors in errors in rotational SO(3) space. */
   public void factoryInSO3Errors(double orientationErrorTolerance)
   {
      isWithinPositionTolerance &= orientationError <= orientationErrorTolerance;
   }

   /** This includes any of the factored in errors of position or orientation. */
   public boolean isWithinPositionTolerance()
   {
      return isWithinPositionTolerance;
   }

   public double getPositionError()
   {
      return positionError;
   }

   public double getOrientationError()
   {
      return orientationError;
   }
}
