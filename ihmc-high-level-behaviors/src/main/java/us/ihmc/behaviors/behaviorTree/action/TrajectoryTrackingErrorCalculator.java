package us.ihmc.behaviors.behaviorTree.action;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

/**
 * This class provides methods for evaluating the tracking error of
 * pose and/or jointspace trajectories executed by the robot.
 */
public class TrajectoryTrackingErrorCalculator extends TrajectoryDurationTracker
{
   private double positionError;
   private double orientationError;
   private double totalAbsoluteJointspaceError;
   private boolean isWithinPositionTolerance;

   public void reset()
   {
      super.reset();

      positionError = Double.NaN;
      orientationError = Double.NaN;
      totalAbsoluteJointspaceError = Double.NaN;
      isWithinPositionTolerance = false;
   }

   public void computePoseTrackingData(FramePose3DReadOnly desired, FramePose3DReadOnly actual)
   {
      positionError = actual.getTranslation().differenceNorm(desired.getTranslation());
      orientationError = actual.getRotation().distance(desired.getRotation(), true);

      isWithinPositionTolerance = true;
   }

   public void resetJointspaceError()
   {
      totalAbsoluteJointspaceError = Double.NaN;
      isWithinPositionTolerance = false;
   }

   /**
    * The user may submit one or more joints' data by calling this sequentially.
    * Absolute errors will be added up.
    */
   public void addJointData(double desired, double actual)
   {
      double singleJointError = Math.abs(actual - desired);

      if (Double.isNaN(totalAbsoluteJointspaceError))
         totalAbsoluteJointspaceError = singleJointError;
      else
         totalAbsoluteJointspaceError += singleJointError;
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

   /** Factors in errors in jointspace. */
   public void factorInJointspaceErrors(double totalErrorTolerance)
   {
      isWithinPositionTolerance = totalAbsoluteJointspaceError <= totalErrorTolerance;
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

   public double getTotalAbsoluteJointspaceError()
   {
      return totalAbsoluteJointspaceError;
   }
}
