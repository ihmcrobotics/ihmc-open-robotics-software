package us.ihmc.quadrupedRobotics.gait;

import static us.ihmc.quadrupedRobotics.gait.QuadrupedGaitState.*;

public enum QuadrupedGaitType
{
   SAFE_WALK(new QuadrupedGaitTransition(ALL_FOURS, 0.00), new QuadrupedGaitTransition(WALK_FRONT_LEFT, 0.05),
             new QuadrupedGaitTransition(ALL_FOURS, 0.25), new QuadrupedGaitTransition(WALK_HIND_LEFT, 0.30),
             new QuadrupedGaitTransition(ALL_FOURS, 0.50), new QuadrupedGaitTransition(WALK_FRONT_RIGHT, 0.55),
             new QuadrupedGaitTransition(ALL_FOURS, 0.75), new QuadrupedGaitTransition(WALK_HIND_RIGHT, 0.80)),
   LATERAL_SEQUENCE_WALK(new QuadrupedGaitTransition(WALK_FRONT_LEFT, 0.00), new QuadrupedGaitTransition(TROT_RIGHT, 0.10),
                         new QuadrupedGaitTransition(WALK_HIND_RIGHT, 0.24), new QuadrupedGaitTransition(PACE_LEFT, 0.30),
                         new QuadrupedGaitTransition(WALK_FRONT_RIGHT, 0.45), new QuadrupedGaitTransition(ALL_FOURS, 0.62),
                         new QuadrupedGaitTransition(WALK_HIND_LEFT, 0.67), new QuadrupedGaitTransition(PACE_RIGHT, 0.90)),
   WALKING_TROT(new QuadrupedGaitTransition(ALL_FOURS, 0.00), new QuadrupedGaitTransition(TROT_RIGHT, 0.10), new QuadrupedGaitTransition(ALL_FOURS, 0.50),
                new QuadrupedGaitTransition(TROT_LEFT, 0.60)),
   RUNNING_TROT(new QuadrupedGaitTransition(TROT_RIGHT, 0.00), new QuadrupedGaitTransition(FLIGHT, 0.40), new QuadrupedGaitTransition(TROT_LEFT, 0.50),
                new QuadrupedGaitTransition(FLIGHT, 0.90)),
   ROTARY_GALLOP(new QuadrupedGaitTransition(HIND_LEFT_ONLY, 0.00), new QuadrupedGaitTransition(BOUND_HIND, 0.10),
                 new QuadrupedGaitTransition(HIND_RIGHT_ONLY, 0.30), new QuadrupedGaitTransition(FLIGHT, 0.40),
                 new QuadrupedGaitTransition(FRONT_RIGHT_ONLY, 0.55), new QuadrupedGaitTransition(BOUND_FRONT, 0.65),
                 new QuadrupedGaitTransition(FRONT_LEFT_ONLY, 0.80), new QuadrupedGaitTransition(FLIGHT, 0.95)),
   TRANSVERSE_GALLOP(new QuadrupedGaitTransition(HIND_LEFT_ONLY, 0.00), new QuadrupedGaitTransition(BOUND_HIND, 0.10),
                     new QuadrupedGaitTransition(HIND_RIGHT_ONLY, 0.30), new QuadrupedGaitTransition(TROT_LEFT, 0.35),
                     new QuadrupedGaitTransition(FRONT_LEFT_ONLY, 0.40), new QuadrupedGaitTransition(BOUND_FRONT, 0.55),
                     new QuadrupedGaitTransition(FRONT_RIGHT_ONLY, 0.65), new QuadrupedGaitTransition(FLIGHT, 0.80)),
   BOUND(new QuadrupedGaitTransition(BOUND_HIND, 0.00), new QuadrupedGaitTransition(FLIGHT, 0.27), new QuadrupedGaitTransition(BOUND_FRONT, 0.40),
         new QuadrupedGaitTransition(FLIGHT, 0.67)),
   ;
   
   private final QuadrupedGait quadrupedGait;
   
   private QuadrupedGaitType(QuadrupedGaitTransition... gaitTransitions)
   {
      this.quadrupedGait = new QuadrupedGait(gaitTransitions);
   }
}
