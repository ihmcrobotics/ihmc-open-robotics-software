package us.ihmc.footstepPlanning.heuristicPlanner;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This planner is intended to support the same footstep planning functionality
 * as in the DRC UI.
 */
public class HeuristicFootstepPlanner
{
   private SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final TurnStraightTurnFootstepGenerator turnStraightTurnFootstepGenerator;

   public HeuristicFootstepPlanner(SideDependentList<? extends ContactablePlaneBody> contactableFeet)
   {
      this.contactableFeet = contactableFeet;

//      turnStraightTurnFootstepGenerator = new TurnStraightTurnFootstepGenerator()
   }

   public FootstepPlan plan()
   {

      return new FootstepPlan(); // TODO
   }
}
