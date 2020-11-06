package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import java.util.ArrayList;
import java.util.List;

public class JumpingGoalHandler
{
   private final List<JumpingGoal> jumpingGoalList = new ArrayList<>();

   private final JumpingParameters parameters;

   public JumpingGoalHandler(JumpingParameters parameters)
   {
      this.parameters = parameters;
   }

   public void consumeJumpingGoal(JumpingGoal jumpingGoal)
   {
      if (isInvalidDuration(jumpingGoal.getSupportDuration()))
         jumpingGoal.setSupportDuration(parameters.getDefaultSupportDuration());
      if (isInvalidDuration(jumpingGoal.getFlightDuration()))
         jumpingGoal.setFlightDuration(parameters.getDefaultFlightDuration());

      jumpingGoalList.add(jumpingGoal);
   }

   private static boolean isInvalidDuration(double duration)
   {
      return Double.isNaN(duration) || duration <= 1e-3;
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
