package us.ihmc.SdfLoader.partNames;

import us.ihmc.tools.FormattingTools;

public enum NeckJointName
{
   LOWER_NECK_PITCH, NECK_YAW, UPPER_NECK_PITCH, NECK_ROLL;

   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case LOWER_NECK_PITCH :
         {
            return "lowerNeckPitch";
         }

         case NECK_YAW :
         {
            return "neckYaw";
         }

         case UPPER_NECK_PITCH :
         {
            return "upperNeckPitch";
         }
         
         case NECK_ROLL :
         {
        	 return "neckRoll";
         }

         default :
         {
            throw new RuntimeException("Should not get to here");
         }
      }
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      return FormattingTools.capitalizeFirstLetter(getCamelCaseNameForStartOfExpression());
   }

   public String toString()
   {
      return getCamelCaseNameForMiddleOfExpression();
   }
}
