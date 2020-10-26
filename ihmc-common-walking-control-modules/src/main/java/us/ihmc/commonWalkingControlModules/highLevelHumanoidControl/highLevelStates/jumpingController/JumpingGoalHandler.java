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

   public boolean hasJumpingGoal()
   {
      return !jumpingGoalList.isEmpty();
   }

   public JumpingGoal pollNextJumpingGoal()
   {
      return jumpingGoalList.remove(0);
   }

   public void pollNextJumpingGoal(JumpingGoal jumpingGoal)
   {
      jumpingGoal.set(jumpingGoalList.remove(0));
   }

   public void peekNextJumpingGoal(JumpingGoal jumpingGoal)
   {
      jumpingGoal.set(jumpingGoalList.get(0));
   }
}
