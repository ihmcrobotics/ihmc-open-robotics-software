package us.ihmc.quadrupedRobotics.gait;

/**
 * Gaits pulled from https://en.wikipedia.org/wiki/Gait Hildebrand graphs.
 * 
 * @author Duncan
 */
public class QuadrupedGait
{
   private final QuadrupedGaitTransition[] gaitTransitions;
   
   public QuadrupedGait(QuadrupedGaitTransition... gaitTransitions)
   {
      this.gaitTransitions = new QuadrupedGaitTransition[gaitTransitions.length];
      
      for (int i = 0; i < gaitTransitions.length; i++)
      {
         this.gaitTransitions[i] = gaitTransitions[i];
      }
   }
}
