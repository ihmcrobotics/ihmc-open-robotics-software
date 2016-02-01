package us.ihmc.commonWalkingControlModules.trajectories;

public class MaximumConstantJerkFinalToeOffAngleComputer
{
   // This class computes the maximum final toe off pitch angle, which can be reached with a given constant jerk (=> cubic polynomial trajectories). 
   // The maximum constant jerk is not entered explicitly, but is one for a (third order polynomial) motion from 0 to the maximumToeOffAngle within a time span of "referenceTime". 
   // If the maximum toe off angle, reachible by the maximum feasible jerk trajectory is bigger then the maximumToeOffAngle, it is cut down to that value. 
   // This leads to trajectories that are either performed with maximum jerk and (most of the times) ending at a final toe off ankle of less then maximumToeOffAngle, or (if there is enough time) 
   // they end at maximumToeOffAngle and are performed with a constant jerk lower than the maximum feasible one. 
   
   private double maximumToeOffAngle; 
   private double referenceTime; 
   
   
   
   public MaximumConstantJerkFinalToeOffAngleComputer()
   {
   }
   
   
   public void reinitialize(double maximumToeOffAngle, double referenceTime)
   {
      this.maximumToeOffAngle = maximumToeOffAngle; 
      this.referenceTime = referenceTime; 
   }
   
   
   public double getMaximumFeasibleConstantJerkFinalToeOffAngle(double initialFootPitchAngle, double toeOffDuration)
   {
      return Math.min(maximumToeOffAngle, initialFootPitchAngle + maximumToeOffAngle * Math.pow(toeOffDuration/referenceTime, 3)); 
   }
   
}
