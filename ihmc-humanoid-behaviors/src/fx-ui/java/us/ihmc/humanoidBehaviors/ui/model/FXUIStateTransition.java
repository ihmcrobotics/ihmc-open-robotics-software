package us.ihmc.humanoidBehaviors.ui.model;

public enum FXUIStateTransition
{
   START,
   POSITION_LEFT_CLICK,
   ORIENTATION_LEFT_CLICK,
   RIGHT_CLICK;

   public boolean isStart()
   {
      return this.equals(START);
   }
}
