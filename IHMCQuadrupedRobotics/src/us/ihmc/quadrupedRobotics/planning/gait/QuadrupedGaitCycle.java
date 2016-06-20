package us.ihmc.quadrupedRobotics.planning.gait;

import static us.ihmc.quadrupedRobotics.planning.gait.QuadrupedSupportConfiguration.*;

import us.ihmc.robotics.robotSide.RobotQuadrant;

/**
 * Gaits pulled from https://en.wikipedia.org/wiki/Gait Hildebrand graphs.
 */
public enum QuadrupedGaitCycle
{
   STAND(new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.00)),
   SAFE_WALK(new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.00), new QuadrupedGaitPhaseTransition(WALK_FRONT_LEFT, 0.05), new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.25),
             new QuadrupedGaitPhaseTransition(WALK_HIND_LEFT, 0.30), new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.50),
             new QuadrupedGaitPhaseTransition(WALK_FRONT_RIGHT, 0.55), new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.75),
             new QuadrupedGaitPhaseTransition(WALK_HIND_RIGHT, 0.80)),
   LATERAL_SEQUENCE_WALK(new QuadrupedGaitPhaseTransition(WALK_FRONT_LEFT, 0.00), new QuadrupedGaitPhaseTransition(TROT_RIGHT, 0.10),
                         new QuadrupedGaitPhaseTransition(WALK_HIND_RIGHT, 0.24), new QuadrupedGaitPhaseTransition(PACE_LEFT, 0.30),
                         new QuadrupedGaitPhaseTransition(WALK_FRONT_RIGHT, 0.45), new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.62),
                         new QuadrupedGaitPhaseTransition(WALK_HIND_LEFT, 0.67), new QuadrupedGaitPhaseTransition(PACE_RIGHT, 0.90)),
   WALKING_TROT(new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.00), new QuadrupedGaitPhaseTransition(TROT_RIGHT, 0.10), new QuadrupedGaitPhaseTransition(ALL_FOURS, 0.50),
                new QuadrupedGaitPhaseTransition(TROT_LEFT, 0.60)),
   PERFECT_TROT(new QuadrupedGaitPhaseTransition(TROT_RIGHT, 0.00), new QuadrupedGaitPhaseTransition(TROT_LEFT, 0.50)),
   RUNNING_TROT(new QuadrupedGaitPhaseTransition(TROT_RIGHT, 0.00), new QuadrupedGaitPhaseTransition(FLIGHT, 0.40), new QuadrupedGaitPhaseTransition(TROT_LEFT, 0.50),
                new QuadrupedGaitPhaseTransition(FLIGHT, 0.90)),
   ROTARY_GALLOP(new QuadrupedGaitPhaseTransition(HIND_LEFT_ONLY, 0.00), new QuadrupedGaitPhaseTransition(BOUND_HIND, 0.10),
                 new QuadrupedGaitPhaseTransition(HIND_RIGHT_ONLY, 0.30), new QuadrupedGaitPhaseTransition(FLIGHT, 0.40),
                 new QuadrupedGaitPhaseTransition(FRONT_RIGHT_ONLY, 0.55), new QuadrupedGaitPhaseTransition(BOUND_FRONT, 0.65),
                 new QuadrupedGaitPhaseTransition(FRONT_LEFT_ONLY, 0.80), new QuadrupedGaitPhaseTransition(FLIGHT, 0.95)),
   TRANSVERSE_GALLOP(new QuadrupedGaitPhaseTransition(HIND_LEFT_ONLY, 0.00), new QuadrupedGaitPhaseTransition(BOUND_HIND, 0.10),
                     new QuadrupedGaitPhaseTransition(HIND_RIGHT_ONLY, 0.30), new QuadrupedGaitPhaseTransition(TROT_LEFT, 0.35),
                     new QuadrupedGaitPhaseTransition(FRONT_LEFT_ONLY, 0.40), new QuadrupedGaitPhaseTransition(BOUND_FRONT, 0.55),
                     new QuadrupedGaitPhaseTransition(FRONT_RIGHT_ONLY, 0.65), new QuadrupedGaitPhaseTransition(FLIGHT, 0.80)),
   BOUND(new QuadrupedGaitPhaseTransition(BOUND_HIND, 0.00), new QuadrupedGaitPhaseTransition(FLIGHT, 0.27), new QuadrupedGaitPhaseTransition(BOUND_FRONT, 0.40),
         new QuadrupedGaitPhaseTransition(FLIGHT, 0.67)),
   ;
   
   private final QuadrupedGaitPhaseTransition[] phaseTransitions;

   private QuadrupedGaitCycle(QuadrupedGaitPhaseTransition... gaitTransitions)
   {
      if (gaitTransitions.length < 1)
      {
         throw new RuntimeException("Must have at least 1 gait transition");
      }
      
      this.phaseTransitions = new QuadrupedGaitPhaseTransition[gaitTransitions.length];
      
      for (int i = 0; i < gaitTransitions.length; i++)
      {
         this.phaseTransitions[i] = gaitTransitions[i];
      }
   }
   
   public QuadrupedSupportConfiguration getGaitPhase(double percentGaitCycle)
   {
      checkPercentGaitCycleForInvalidValue(percentGaitCycle);
      
      return phaseTransitions[getIndexOfCurrentPhase(percentGaitCycle)].getGaitState();
   }
   
   public QuadrupedSupportConfiguration getNextGaitPhase(double percentGaitCycle)
   {
      checkPercentGaitCycleForInvalidValue(percentGaitCycle);
      
      return phaseTransitions[getIndexOfNextPhase(percentGaitCycle)].getGaitState();
   }
   
   public double getRemainingPhaseDuration(double percentGaitCycle)
   {
      checkPercentGaitCycleForInvalidValue(percentGaitCycle);
      
      int currentStateIndex = getIndexOfCurrentPhase(percentGaitCycle);
      
      int nextIndex = (currentStateIndex + 1) % phaseTransitions.length;
      
      if (nextIndex > currentStateIndex)
      {
         return phaseTransitions[nextIndex].getTransitionTime() - percentGaitCycle;
      }
      else
      {
         return phaseTransitions[nextIndex].getTransitionTime() + (1.0 - percentGaitCycle);
      }
   }
   
   public double getRemainingSwingDuration(RobotQuadrant robotQuadrant, double percentGaitCycle)
   {
      checkPercentGaitCycleForInvalidValue(percentGaitCycle);
      
      int currentStateIndex = getIndexOfCurrentPhase(percentGaitCycle);
      
      if (!phaseTransitions[currentStateIndex].getGaitState().isSwingQuadrant(robotQuadrant))
      {
         throw new RuntimeException("quadrant "+ robotQuadrant + " is not a swing leg");
      }
      
      int i = currentStateIndex + 1;
      i %= phaseTransitions.length;
      while (true)
      {
         if (phaseTransitions[i].getGaitState().isSupportQuadrant(robotQuadrant))
         {
            break;
         }
         else if (i == currentStateIndex)
         {
            return Double.POSITIVE_INFINITY;
         }
         else
         {
            i++;
            i %= phaseTransitions.length;
         }
      }
      
      if (i > currentStateIndex)
      {
         return phaseTransitions[i].getTransitionTime() - percentGaitCycle;
      }
      else
      {
         return phaseTransitions[i].getTransitionTime() + (1.0 - percentGaitCycle);
      }
   }

   private int getIndexOfCurrentPhase(double percentGaitCycle)
   {
      int i;
      for (i = 0; i < phaseTransitions.length && phaseTransitions[i].getTransitionTime() <= percentGaitCycle; i++);
      return i - 1;
   }
   
   private int getIndexOfNextPhase(double percentGaitCycle)
   {
      return (getIndexOfCurrentPhase(percentGaitCycle) + 1) % phaseTransitions.length;
   }

   private void checkPercentGaitCycleForInvalidValue(double percentGaitCycle)
   {
      if (percentGaitCycle >= 1.0 || percentGaitCycle < 0.0 || Double.isNaN(percentGaitCycle))
      {
         throw new RuntimeException("percentGaitCycle must be 0.0 < 1.0. percentGaitCycle: " + percentGaitCycle);
      }
   }

   public RobotQuadrant[] getSupportQuadrants(double percentGaitCycle)
   {
      return getGaitPhase(percentGaitCycle).supportQuadrants();
   }

   public RobotQuadrant[] getSwingQuadrants(double percentGaitCycle)
   {
      return getGaitPhase(percentGaitCycle).swingQuadrants();
   }

   public boolean isLastPhase(double percentGaitCycle)
   {
      return getIndexOfCurrentPhase(percentGaitCycle) == phaseTransitions.length - 1;
   }
}
