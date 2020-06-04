package us.ihmc.footstepPlanning.communication;

public enum UIStepAdjustmentFrame
{
   WORLD, LOCAL;

   public UIStepAdjustmentFrame getOppositeMode()
   {
      if (this == WORLD)
         return LOCAL;
      else
         return WORLD;
   }

   public static UIStepAdjustmentFrame getDefault()
   {
      return LOCAL;
   }
}
