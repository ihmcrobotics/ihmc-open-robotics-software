package us.ihmc.quadrupedRobotics.planning.gait;

public class QuadrupedGaitPhaseTransition
{
   private final QuadrupedSupportConfiguration supportConfiguration;
   /** Transition time is the time ratio from 0.0 to 1.0 (not including 1.0) where the transition occurs. */
   private final double transitionTime;
   
   public QuadrupedGaitPhaseTransition(QuadrupedSupportConfiguration gaitState, double transitionTime)
   {
      if (transitionTime >= 1.0)
      {
         throw new RuntimeException("transitionTime must be <= 1.0");
      }
      
      this.supportConfiguration = gaitState;
      this.transitionTime = transitionTime;
   }
   
   public QuadrupedSupportConfiguration getGaitState()
   {
      return supportConfiguration;
   }
   
   public double getTransitionTime()
   {
      return transitionTime;
   }
}
