package us.ihmc.SdfLoader.partNames;

import org.apache.commons.lang3.StringUtils;

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
      return StringUtils.capitalize(getCamelCaseNameForStartOfExpression());
   }

   public String toString()
   {
      return getCamelCaseNameForMiddleOfExpression();
   }
}
