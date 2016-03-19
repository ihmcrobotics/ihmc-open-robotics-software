package us.ihmc.quadrupedRobotics.gait;

public class QuadrupedGaitTransition
{
   private final QuadrupedGaitState gaitState;
   /** Transition time is the time ratio from 0.0 to 1.0 (not including 1.0) where the transition occurs. */
   private final double transitionTime;
   
   public QuadrupedGaitTransition(QuadrupedGaitState gaitState, double transitionTime)
   {
      if (transitionTime >= 1.0)
      {
         throw new RuntimeException("transitionTime must be <= 1.0");
      }
      
      this.gaitState = gaitState;
      this.transitionTime = transitionTime;
   }
   
   public QuadrupedGaitState getGaitState()
   {
      return gaitState;
   }
   
   public double getTransitionTime()
   {
      return transitionTime;
   }
}
