package us.ihmc.footstepPlanning.communication;

public enum UIStepAdjustmentMode
{
   TRANSLATION, ORIENTATION;

   public UIStepAdjustmentMode getOppositeMode()
   {
      if (this == TRANSLATION)
         return ORIENTATION;
      else
         return TRANSLATION;
   }
}
