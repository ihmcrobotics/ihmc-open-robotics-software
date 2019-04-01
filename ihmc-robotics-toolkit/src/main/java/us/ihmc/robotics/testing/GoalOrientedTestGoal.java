package us.ihmc.robotics.testing;

public abstract class GoalOrientedTestGoal
{
   private boolean hasMetGoal = false;

   protected void update()
   {
      if (!hasMetGoal && currentlyMeetsGoal())
      {
         hasMetGoal = true;
      }
   }

   public boolean hasMetGoal()
   {
      return hasMetGoal;
   }

   public void reset()
   {
      hasMetGoal = false;
   }

   public abstract boolean currentlyMeetsGoal();

   public abstract String toString();
}
