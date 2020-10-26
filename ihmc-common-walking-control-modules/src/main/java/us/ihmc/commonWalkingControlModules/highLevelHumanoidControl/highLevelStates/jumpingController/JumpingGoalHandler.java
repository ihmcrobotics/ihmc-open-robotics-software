package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import java.util.ArrayList;
import java.util.List;

public class JumpingGoalHandler
{
   private final List<JumpingGoal> jumpingGoalList = new ArrayList<>();

   public void consumeJumpingGoal(JumpingGoal jumpingGoal)
   {
      jumpingGoalList.add(jumpingGoal);
   }
}
