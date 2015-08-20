package us.ihmc.SdfLoader.partNames;

import us.ihmc.tools.FormattingTools;

public enum SpineJointName
{
   SPINE_PITCH, SPINE_YAW, SPINE_ROLL;

   public static final SpineJointName[] values = values();

   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case SPINE_PITCH :
         {
            return "spinePitch";
         }

         case SPINE_YAW :
         {
            return "spineYaw";
         }

         case SPINE_ROLL :
         {
            return "spineRoll";
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
