package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.StringUtils;

public enum ArmJointName
{
   SHOULDER_YAW, SHOULDER_ROLL, SHOULDER_PITCH, ELBOW_PITCH, WRIST_ROLL, FIRST_WRIST_PITCH, SECOND_WRIST_PITCH, ELBOW_ROLL, ELBOW_YAW, WRIST_YAW;

   public static final ArmJointName[] values = values();
   
   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case SHOULDER_YAW :
         {
            return "shoulderYaw";
         }

         case SHOULDER_ROLL :
         {
            return "shoulderRoll";
         }

         case SHOULDER_PITCH :
         {
            return "shoulderPitch";
         }

         case ELBOW_PITCH :
         {
            return "elbowPitch";
         }
         
         case ELBOW_ROLL :
         {
            return "elbowRoll";
         }
         
         case ELBOW_YAW :
         { 
            return "elbowYaw";
         }
         
         case WRIST_ROLL :
         {
            return "wristRoll";
         }
         
         case FIRST_WRIST_PITCH:
         {
            return "firstWristPitch";
         }

         case SECOND_WRIST_PITCH:
         {
            return "secondWristPitch";
         }
         
         case WRIST_YAW:
            return "wristYaw";

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
