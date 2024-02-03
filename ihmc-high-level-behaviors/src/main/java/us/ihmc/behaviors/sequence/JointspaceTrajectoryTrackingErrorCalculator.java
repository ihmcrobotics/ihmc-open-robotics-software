package us.ihmc.behaviors.sequence;

/**
 * This class provides methods for evaluating the tracking error of
 * jointspace trajectories executed by the robot.
 */
public class JointspaceTrajectoryTrackingErrorCalculator extends TrajectoryDurationTracker
{
   private double totalAbsolutePositionError;
   private boolean isWithinPositionTolerance;

   public void reset()
   {
      super.reset();

      totalAbsolutePositionError = Double.NaN;
      isWithinPositionTolerance = false;
   }

   /**
    * The user may submit one or more joints' data by calling this sequentially.
    * Absolute errors will be added up.
    */
   public void addJointData(double desired, double actual)
   {
      double singleJointError = Math.abs(actual - desired);

      if (Double.isNaN(totalAbsolutePositionError))
         totalAbsolutePositionError = singleJointError;
      else
         totalAbsolutePositionError += singleJointError;
   }

   public void applyTolerance(double totalErrorTolerance)
   {
      isWithinPositionTolerance = totalAbsolutePositionError <= totalErrorTolerance;
   }

   public boolean isWithinPositionTolerance()
   {
      return isWithinPositionTolerance;
   }

   public double getTotalAbsolutePositionError()
   {
      return totalAbsolutePositionError;
   }
}
