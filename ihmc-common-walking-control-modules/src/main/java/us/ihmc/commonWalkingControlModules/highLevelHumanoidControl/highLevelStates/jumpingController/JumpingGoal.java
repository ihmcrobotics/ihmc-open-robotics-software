package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.communication.controllerAPI.command.Command;

public class JumpingGoal implements Command<JumpingGoal, JumpingGoal>
{
   private double goalLength;
   private double goalFootWidth;
   private double goalRotation;
   private double goalHeight;

   private double supportDuration;
   private double flightDuration;

   public JumpingGoal()
   {
      clear();
   }

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

   public void setSupportDuration(double supportDuration)
   {
      this.supportDuration = supportDuration;
   }

   public void setFlightDuration(double flightDuration)
   {
      this.flightDuration = flightDuration;
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

   public double getSupportDuration()
   {
      return supportDuration;
   }

   public double getFlightDuration()
   {
      return flightDuration;
   }

   @Override
   public void clear()
   {
      goalLength = Double.NaN;
      goalFootWidth = Double.NaN;
      goalRotation = Double.NaN;
      goalHeight = Double.NaN;
      supportDuration = Double.NaN;
      flightDuration = Double.NaN;
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
      setSupportDuration(other.getSupportDuration());
      setFlightDuration(other.getFlightDuration());
   }
}
