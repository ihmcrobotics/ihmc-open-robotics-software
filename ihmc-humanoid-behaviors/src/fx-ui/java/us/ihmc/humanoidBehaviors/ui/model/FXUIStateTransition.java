package us.ihmc.humanoidBehaviors.ui.model;

public enum FXUIStateTransition
{
   START,
   SNAPPED_POSITION_LEFT_CLICK,
   SNAPPED_POSITION_RIGHT_CLICK;

   public boolean isStart()
   {
      return this.equals(START);
   }
}
