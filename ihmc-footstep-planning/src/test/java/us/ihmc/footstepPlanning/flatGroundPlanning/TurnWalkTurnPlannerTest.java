package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;

public class TurnWalkTurnPlannerTest extends FootstepPlannerOnFlatGroundTest
{
   private static final boolean visualize = false;
   private static final boolean keepUp = false;
   private final TurnWalkTurnPlanner planner = new TurnWalkTurnPlanner();

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

   @Override
   public boolean keepUp()
   {
      return keepUp;
   }
}
