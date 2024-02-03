package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.tools.NonWallTimer;

/**
 * This class provides methods for evaluating the tracking error of
 * pose trajectories executed by the robot.
 */
public class TaskspaceTrajectoryTrackingErrorCalculator
{
   private double positionError;
   private double orientationError;
   private boolean timeIsUp;
   private boolean hitTimeLimit;
   private boolean isWithinPositionTolerance;
   private final NonWallTimer executionTimer = new NonWallTimer();

   public void update(double robotTime)
   {
      executionTimer.update(robotTime);
   }

   public void reset()
   {
      executionTimer.reset();

      positionError = Double.POSITIVE_INFINITY;
      orientationError = Double.POSITIVE_INFINITY;
      timeIsUp = false;
      hitTimeLimit = false;
      isWithinPositionTolerance = false;
   }

   public void computeExecutionTimings(double nominalExecutionDuration)
   {
      computeExecutionTimings(nominalExecutionDuration, nominalExecutionDuration * 1.5);
   }

   public void computeExecutionTimings(double nominalExecutionDuration, double timeout)
   {
      timeIsUp = !executionTimer.isRunning(nominalExecutionDuration);
      // Default timeout is 50% longer than nominal TODO: Introduce parameter
      hitTimeLimit = !executionTimer.isRunning(timeout);
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

   public boolean getTimeIsUp()
   {
      return timeIsUp;
   }

   public boolean getHitTimeLimit()
   {
      return hitTimeLimit;
   }

   /** This includes any of the factored in errors of position or orientation. */
   public boolean isWithinPositionTolerance()
   {
      return isWithinPositionTolerance;
   }

   public double getElapsedTime()
   {
      return executionTimer.getElapsedTime();
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
