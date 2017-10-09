package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.StringUtils;

public enum NeckJointName
{
   PROXIMAL_NECK_YAW, PROXIMAL_NECK_PITCH, PROXIMAL_NECK_ROLL, DISTAL_NECK_YAW, DISTAL_NECK_PITCH, DISTAL_NECK_ROLL;

   /**
    * @deprecated Do not iterate over these values!
    */
   public static final NeckJointName[] values = values();
   
   public String getUnderBarName()
   {
      switch (this)
      {
      case PROXIMAL_NECK_YAW:
         return "proximal_neck_yaw";
      case PROXIMAL_NECK_PITCH:
         return "proximal_neck_pitch";
      case PROXIMAL_NECK_ROLL:
         return "proximal_neck_roll";
      case DISTAL_NECK_YAW:
         return "distal_neck_yaw";
      case DISTAL_NECK_PITCH:
         return "distal_neck_pitch";
      case DISTAL_NECK_ROLL:
         return "distal_neck_roll";
      default:
         throw new RuntimeException("Should not get to here");
      }
   }
   
   public String getCamelCaseName()
   {
      switch (this)
      {
      case PROXIMAL_NECK_YAW:
         return "proximalNeckYaw";
      case PROXIMAL_NECK_PITCH:
         return "proximalNeckPitch";
      case PROXIMAL_NECK_ROLL:
         return "proximalNeckRoll";
      case DISTAL_NECK_YAW:
         return "distalNeckYaw";
      case DISTAL_NECK_PITCH:
         return "distalNeckPitch";
      case DISTAL_NECK_ROLL:
         return "distalNeckRoll";
      default:
         throw new RuntimeException("Should not get to here");
      }
   }
   
   /**
    * @deprecated Use getCamelCaseName() instead.
    */
   public String getCamelCaseNameForStartOfExpression()
   {
      return getCamelCaseName();
   }
   
   public String getPascalCaseName()
   {
      return StringUtils.capitalize(getCamelCaseName());
   }

   /**
    * @deprecated Use getPascalCaseName() instead.
    */
   public String getCamelCaseNameForMiddleOfExpression()
   {
      return getPascalCaseName();
   }
}
