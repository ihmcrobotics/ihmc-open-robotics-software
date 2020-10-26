package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.communication.controllerAPI.command.Command;

public class JumpingGoal implements Command<JumpingGoal, JumpingGoal>
{
   private double goalLength;
   private double goalFootWidth;
   private double goalRotation;
   private double goalHeight;

   public void setGoalLength(double goalLength)
   {
      this.goalLength = goalLength;
   }

   public void setGoalFootWidth(double goalFootWidth)
   {
      this.goalFootWidth = goalFootWidth;
   }

   public void setGoalRotation(double goalRotation)
   {
      this.goalRotation = goalRotation;
   }

   public void setGoalHeight(double goalHeight)
   {
      this.goalHeight = goalHeight;
   }

   public double getGoalLength()
   {
      return goalLength;
   }

   public double getGoalFootWidth()
   {
      return goalFootWidth;
   }

   public double getGoalRotation()
   {
      return goalRotation;
   }

   public double getGoalHeight()
   {
      return goalHeight;
   }

   @Override
   public void clear()
   {
      goalLength = Double.NaN;
      goalFootWidth = Double.NaN;
      goalRotation = Double.NaN;
      goalHeight = Double.NaN;
   }

   @Override
   public void setFromMessage(JumpingGoal message)
   {
      set(message);
   }

   @Override
   public Class<JumpingGoal> getMessageClass()
   {
      return JumpingGoal.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return -1;
   }

   @Override
   public void set(JumpingGoal other)
   {
      setGoalLength(other.getGoalLength());
      setGoalFootWidth(other.getGoalFootWidth());
      setGoalRotation(other.getGoalRotation());
      setGoalHeight(other.getGoalHeight());
   }
}
