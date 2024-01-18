package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.tools.NonWallTimer;

/**
 * This class provides methods for evaluating the tracking error of
 * pose trajectories executed by the robot.
 */
public class TrajectoryTrackingErrorCalculator
{
   private double positionErrorDistance;
   private double orientationErrorDistance;

   private boolean timeIsUp;
   private boolean hitTimeLimit;
   private boolean isWithinPositionTolerance;
   private double robotTime;
   private final NonWallTimer executionTimer = new NonWallTimer();

   private final PoseDerivativeCalculator poseDerivativeCalculator = new PoseDerivativeCalculator();

   public void update(double robotTime)
   {
      this.robotTime = robotTime;
      executionTimer.update(robotTime);
   }

   public void reset()
   {
      executionTimer.reset();

      positionErrorDistance = Double.POSITIVE_INFINITY;
      orientationErrorDistance = Double.POSITIVE_INFINITY;
      timeIsUp = false;
      hitTimeLimit = false;
      isWithinPositionTolerance = false;

      poseDerivativeCalculator.reset();
   }

   public void computeExecutionTimings(double nominalExecutionDuration)
   {
      timeIsUp = !executionTimer.isRunning(nominalExecutionDuration);
      // Default timeout is 50% longer than nominal TODO: Introduce parameter
      hitTimeLimit = !executionTimer.isRunning(nominalExecutionDuration * 1.5);
   }

   public void computePoseTrackingData(FramePose3DReadOnly desired, FramePose3DReadOnly actual)
   {
      positionErrorDistance = actual.getTranslation().differenceNorm(desired.getTranslation());
      orientationErrorDistance = actual.getRotation().distance(desired.getRotation(), true);

      poseDerivativeCalculator.compute(actual, robotTime);

      isWithinPositionTolerance = true;
   }

   /** Factors in errors in x,z,y translational Euclidean (R3) space. */
   public void factorInR3Errors(double positionErrorTolerance)
   {
      isWithinPositionTolerance &= positionErrorDistance <= positionErrorTolerance;
   }

   /** Factors in errors in rotational SO(3) space. */
   public void factoryInSO3Errors(double orientationErrorTolerance)
   {
      isWithinPositionTolerance &= orientationErrorDistance <= orientationErrorTolerance;
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

   public double getPositionErrorDistance()
   {
      return positionErrorDistance;
   }

   public double getOrientationErrorDistance()
   {
      return orientationErrorDistance;
   }

   public double getLinearVelocity()
   {
      return poseDerivativeCalculator.getLinearVelocity().norm();
   }
}
